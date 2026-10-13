package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Collider;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting;
import org.firstinspires.ftc.teamcode.simcore.ShotSettings;
import org.firstinspires.ftc.teamcode.simcore.Surface;
import org.firstinspires.ftc.teamcode.simcore.Turn;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.firstinspires.ftc.teamcode.simcore.WheelPair;
import org.firstinspires.ftc.teamcode.simcore.World;

final class SimLaunches {
    static final int MOST_KEPT = 30;
    static final long MOST_KEPT_BYTES = 24L * 1024 * 1024;
    static final String SETTINGS_KEPT = "settings";
    static final String LAUNCHED_SO_FAR = "launched";
    static final String REMEMBERED_FILE = "launch-settings.json";
    static final double CHUNK_SECONDS = 0.1;
    static final String POLLEN_COLOUR = "#ffef3f";

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final Field FIELD = SimPlacement.FIELD;
    private static final String[] PHASES = {"f", "s", "r", "x"};

    private record Launch(int id, ShotSettings settings, JsonObject summary, String shot, String frames) {}

    private sealed interface Asked permits Asked.Settled, Asked.Refused {
        record Settled(ShotSettings settings) implements Asked {}

        record Refused(JsonObject refusal) implements Asked {}

        static Asked of(ShotSettings settings) {
            return new Settled(settings);
        }

        static Asked refused(String setting, String rule) {
            JsonObject refusal = new JsonObject();
            refusal.add("setting", setting == null ? JsonNull.INSTANCE : new JsonPrimitive(setting));
            refusal.addProperty("rule", rule);
            return new Refused(refusal);
        }
    }

    private final Path file;
    private final Store store;
    private final Clock clock;
    private final AtomicBoolean simulating = new AtomicBoolean();
    private final List<Launch> launches = new ArrayList<>();
    private int launched;
    private ShotSettings remembered = ShotSettings.defaults();
    private String forgot = "";

    SimLaunches(Path file, Store store, Clock clock) {
        this.file = file;
        this.store = store;
        this.clock = clock;
        recall();
    }

    private void recall() {
        Optional<byte[]> bytes;
        try {
            bytes = store.readIfThere(file);
        } catch (RuntimeException e) {
            forgot = "The launch settings kept in " + file + " could not be read (" + e.getMessage()
                    + "), so the form starts from the defaults.";
            return;
        }
        if (bytes.isEmpty()) {
            return;
        }
        JsonObject kept;
        try {
            kept = GSON.fromJson(new String(bytes.get(), StandardCharsets.UTF_8), JsonObject.class);
        } catch (RuntimeException e) {
            forgot = "The launch settings kept in " + file + " are not settings (" + e.getMessage()
                    + "), so the form starts from the defaults.";
            return;
        }
        if (kept == null) {
            return;
        }
        Optional<Double> count = numberIn(kept.get(LAUNCHED_SO_FAR));
        if (count.isPresent() && count.get() >= 0 && count.get() < Integer.MAX_VALUE) {
            launched = (int) Math.floor(count.get());
        }
        JsonElement settingsKept = kept.get(SETTINGS_KEPT);
        JsonObject asKept =
                settingsKept != null && settingsKept.isJsonObject() ? settingsKept.getAsJsonObject() : new JsonObject();
        Map<ShotSetting, Double> set = new EnumMap<>(ShotSetting.class);
        List<String> dropped = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : asKept.entrySet()) {
            Checked<ShotSetting> named = ShotSetting.named(entry.getKey());
            if (!(named instanceof Checked.Ok<ShotSetting> ok)) {
                dropped.add(entry.getKey() + " (there is no such setting now)");
                continue;
            }
            Optional<Double> number = numberIn(entry.getValue());
            if (number.isEmpty()) {
                dropped.add(entry.getKey() + " (kept as " + entry.getValue() + ", which is not a number)");
                continue;
            }
            Checked<Double> admitted = ok.value().admits(number.get());
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                dropped.add(entry.getKey() + " (" + rejected.rule() + ")");
                continue;
            }
            set.put(ok.value(), number.get());
        }
        Checked<ShotSettings> settings = ShotSettings.of(set);
        if (settings instanceof Checked.Ok<ShotSettings> ok) {
            remembered = ok.value();
        } else {
            dropped.add(rule(settings));
        }
        if (!dropped.isEmpty()) {
            forgot = "Went back to the default for what was kept of " + String.join(", ", dropped) + ".";
        }
    }

    private static Optional<Double> numberIn(JsonElement value) {
        if (value == null
                || !value.isJsonPrimitive()
                || !value.getAsJsonPrimitive().isNumber()) {
            return Optional.empty();
        }
        return Optional.of(value.getAsDouble());
    }

    private static String rule(Checked<?> checked) {
        return checked.fold(value -> "", rule -> rule);
    }

    Response settings() {
        ShotSettings now;
        String said;
        synchronized (this) {
            now = remembered;
            said = forgot;
        }
        JsonArray groups = new JsonArray();
        for (ShotSetting.Group group : ShotSetting.Group.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", group.asked());
            one.addProperty("label", group.label());
            one.addProperty("says", group.says());
            one.addProperty("open", group.openByDefault());
            groups.add(one);
        }
        JsonArray listed = new JsonArray();
        for (ShotSetting setting : ShotSetting.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", setting.asked());
            one.addProperty("group", setting.group().asked());
            one.addProperty("label", setting.label());
            one.addProperty("unit", setting.unit());
            one.addProperty("says", setting.says());
            one.addProperty("least", setting.least());
            one.addProperty("most", setting.most());
            one.addProperty("byDefault", setting.byDefault());
            one.addProperty("switch", setting.kind() instanceof ShotSetting.Switch);
            one.addProperty("value", now.value(setting));
            if (setting.kind() instanceof ShotSetting.OnSurface on) {
                one.addProperty("surface", on.surface().asked());
                one.addProperty("property", on.property().asked());
            }
            listed.add(one);
        }
        JsonArray surfaces = new JsonArray();
        for (Surface surface : Surface.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", surface.asked());
            one.addProperty("label", surface.label());
            one.addProperty("says", surface.says());
            surfaces.add(one);
        }
        JsonObject listing = new JsonObject();
        listing.add("groups", groups);
        listing.add("settings", listed);
        listing.add("surfaces", surfaces);
        listing.addProperty("forgot", said);
        return Response.json(GSON.toJson(listing));
    }

    private static Asked asked(String body) {
        JsonObject json;
        try {
            json = GSON.fromJson(body == null || body.isBlank() ? "{}" : body, JsonObject.class);
        } catch (RuntimeException e) {
            return Asked.refused(
                    null,
                    "a launch is asked for with an object of numbers by setting, as" + " {\"pitch\": 60}, not " + body);
        }
        if (json == null) {
            return Asked.of(ShotSettings.defaults());
        }
        Map<ShotSetting, Double> set = new EnumMap<>(ShotSetting.class);
        for (Map.Entry<String, JsonElement> entry : json.entrySet()) {
            Checked<ShotSetting> named = ShotSetting.named(entry.getKey());
            if (!(named instanceof Checked.Ok<ShotSetting> ok)) {
                return Asked.refused(entry.getKey(), rule(named));
            }
            Optional<Double> number = numberIn(entry.getValue());
            if (number.isEmpty()) {
                return Asked.refused(
                        entry.getKey(),
                        ok.value().label() + " is a number of " + ok.value().unit() + ", not " + entry.getValue());
            }
            Checked<Double> admitted = ok.value().admits(number.get());
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                return Asked.refused(entry.getKey(), rejected.rule());
            }
            set.put(ok.value(), number.get());
        }
        Checked<ShotSettings> settings = ShotSettings.of(set);
        if (!(settings instanceof Checked.Ok<ShotSettings> ok)) {
            return Asked.refused(null, rule(settings));
        }
        return Asked.of(ok.value());
    }

    private static Response refusedAs(Asked asked) {
        return Response.json(
                400, GSON.toJson(asked instanceof Asked.Refused refused ? refused.refusal() : new JsonObject()));
    }

    Response preview(String body) {
        Asked asked = asked(body);
        if (!(asked instanceof Asked.Settled settled)) {
            return refusedAs(asked);
        }
        Checked<Shot.Preview> previewed = Shot.preview(FIELD, settled.settings());
        if (!(previewed instanceof Checked.Ok<Shot.Preview> ok)) {
            return refusedAs(Asked.refused(null, rule(previewed)));
        }
        Shot.Preview preview = ok.value();
        WheelPair.Launched launch = preview.launched();
        JsonObject json = new JsonObject();
        json.addProperty("speed", launch.velocity().length());
        json.addProperty("spinRpm", preview.spinRpm());
        json.addProperty("spinKind", preview.kind().name().toLowerCase(Locale.ROOT));
        json.addProperty("spinParameter", preview.spinParameter());
        json.add("surfaces", pair(launch.surfaceOne(), launch.surfaceTwo()));
        json.add("after", pair(launch.surfaceOneAfter(), launch.surfaceTwoAfter()));
        json.add("ticksPerSecond", pair(preview.ticksPerSecondOne(), preview.ticksPerSecondTwo()));
        json.add("at", vector(launch.at(), 4));
        json.add(
                "direction",
                vector(launch.velocity().length() > 0 ? launch.velocity().unit() : Vec3.zero(), 6));
        JsonArray aims = new JsonArray();
        for (Shot.Aim aim : preview.aims()) {
            JsonObject one = new JsonObject();
            one.addProperty("cell", aim.cell());
            one.addProperty("alliance", aim.alliance());
            one.addProperty("upturned", aim.upturned());
            one.addProperty("yaw", aim.yawDegrees());
            one.addProperty("distance", aim.distance());
            one.addProperty("height", aim.height());
            aims.add(one);
        }
        json.add("aims", aims);
        return Response.json(GSON.toJson(json));
    }

    Response launch(String body) {
        Asked asked = asked(body);
        if (!(asked instanceof Asked.Settled settled)) {
            return refusedAs(asked);
        }
        ShotSettings settings = settled.settings();
        if (!simulating.compareAndSet(false, true)) {
            return Response.error(409, "a launch is already being simulated; it takes a moment");
        }
        try {
            long began = clock.nanos();
            Checked<Shot.Flying> flying = Shot.from(FIELD, settings);
            if (!(flying instanceof Checked.Ok<Shot.Flying> ok)) {
                return refusedAs(Asked.refused(null, rule(flying)));
            }
            Shot shot = ok.value();
            double until = 0;
            while (shot instanceof Shot.Flying still) {
                until += CHUNK_SECONDS;
                shot = still.advance(seconds(until));
            }
            Shot.Finished finished = (Shot.Finished) shot;
            double spent = (clock.nanos() - began) / 1e9;
            return Response.json(GSON.toJson(kept(settings, finished, spent)));
        } finally {
            simulating.set(false);
        }
    }

    private synchronized JsonObject kept(ShotSettings settings, Shot.Finished finished, double spent) {
        int id = ++launched;
        long at = clock.millisSinceEpoch();
        Optional<ShotSettings> before = launches.isEmpty()
                ? Optional.empty()
                : Optional.of(launches.get(launches.size() - 1).settings());
        JsonObject summary = summary(id, at, settings, before, finished, spent);
        JsonObject shot = shotJson(id, at, settings, finished, spent);
        launches.add(new Launch(id, settings, summary, GSON.toJson(shot), csv(finished)));
        while (launches.size() > MOST_KEPT || (launches.size() > 1 && keptBytes() > MOST_KEPT_BYTES)) {
            launches.remove(0);
        }
        remembered = settings;
        forgot = "";
        JsonObject kept = new JsonObject();
        kept.add(SETTINGS_KEPT, changed(settings));
        kept.addProperty(LAUNCHED_SO_FAR, launched);
        try {
            store.writeWhole(file, GSON.toJson(kept).getBytes(StandardCharsets.UTF_8));
        } catch (RuntimeException e) {
            forgot = "These settings could not be kept for next time: " + e.getMessage();
        }
        JsonObject answer = new JsonObject();
        answer.addProperty("id", id);
        return answer;
    }

    private long keptBytes() {
        long bytes = 0;
        for (Launch launch : launches) {
            bytes += launch.shot().length() + launch.frames().length();
        }
        return bytes;
    }

    static boolean isALaunchNumber(String id) {
        return id != null && id.matches("[0-9]{1,9}");
    }

    synchronized Response listing() {
        JsonArray listed = new JsonArray();
        for (int i = launches.size() - 1; i >= 0; i--) {
            listed.add(launches.get(i).summary());
        }
        JsonObject listing = new JsonObject();
        listing.add("launches", listed);
        return Response.json(GSON.toJson(listing));
    }

    private synchronized Optional<Launch> find(String id) {
        if (!isALaunchNumber(id)) {
            return Optional.empty();
        }
        for (Launch launch : launches) {
            if (String.valueOf(launch.id()).equals(id)) {
                return Optional.of(launch);
            }
        }
        return Optional.empty();
    }

    Response shot(String id) {
        return find(id).map(launch -> Response.json(launch.shot()))
                .orElse(Response.error(404, "no such launch: " + id));
    }

    Response frames(String id) {
        return find(id).map(launch -> new Response(200, "text/csv; charset=utf-8", launch.frames()))
                .orElse(Response.error(404, "no such launch: " + id));
    }

    Response viewer() {
        return Response.html(SimLaunchPage.viewer());
    }

    static Response toTheViewer(String from, String id) {
        if (!isALaunchNumber(id)) {
            return Response.error(404, "a launch is a number, and that is not one");
        }
        return new Response(308, "text/plain; charset=utf-8", "").withHeader("Location", from + "#" + id);
    }

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static JsonObject changed(ShotSettings settings) {
        JsonObject changed = new JsonObject();
        for (Map.Entry<ShotSetting, Double> entry : settings.changed().entrySet()) {
            changed.addProperty(entry.getKey().asked(), entry.getValue());
        }
        return changed;
    }

    private static JsonObject since(ShotSettings before, ShotSettings now) {
        JsonObject since = new JsonObject();
        for (ShotSetting setting : ShotSetting.values()) {
            if (Double.compare(before.value(setting), now.value(setting)) != 0) {
                since.add(setting.asked(), pair(before.value(setting), now.value(setting)));
            }
        }
        return since;
    }

    private static JsonArray pair(double one, double two) {
        JsonArray pair = new JsonArray();
        pair.add(one);
        pair.add(two);
        return pair;
    }

    private static double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    private static JsonArray vector(Vec3 v, int places) {
        JsonArray xyz = new JsonArray();
        xyz.add(round(v.x(), places));
        xyz.add(round(v.y(), places));
        xyz.add(round(v.z(), places));
        return xyz;
    }

    private static JsonObject ending(Shot.Finish finish) {
        JsonObject ending = new JsonObject();
        ending.addProperty("kind", finish.kind());
        ending.addProperty("says", finish.says());
        ending.addProperty("t", round(finish.t().value(), 5));
        if (finish instanceof Shot.Finish.AtRest rest) {
            ending.add("at", vector(rest.at(), 4));
        } else if (finish instanceof Shot.Finish.LeftTheField out) {
            ending.add("at", vector(out.at(), 4));
        } else {
            ending.add("at", JsonNull.INSTANCE);
        }
        return ending;
    }

    private static JsonObject summary(
            int id,
            long at,
            ShotSettings settings,
            Optional<ShotSettings> before,
            Shot.Finished finished,
            double spent) {
        Shot.Story story = finished.story();
        WheelPair.Launched launch = story.launched();
        double simulated = story.simulated().value();
        JsonObject summary = new JsonObject();
        summary.addProperty("id", id);
        summary.addProperty("at", at);
        summary.add("changed", changed(settings));
        summary.add("sinceBefore", before.map(was -> since(was, settings)).orElse(new JsonObject()));
        summary.add("ending", ending(finished.finish()));
        summary.addProperty("seconds", round(simulated, 5));
        summary.addProperty("computeSeconds", spent);
        summary.addProperty("realTime", simulated > 0 ? spent / simulated : 0);
        summary.addProperty("impacts", story.impacts().size());
        summary.addProperty("speed", round(launch.velocity().length(), 3));
        summary.addProperty("spinRpm", round(launch.spinRpm(), 1));
        summary.addProperty("spinKind", launch.kind().name().toLowerCase(Locale.ROOT));
        summary.addProperty("apex", round(story.apex().pollen().at().z(), 3));
        summary.add(
                "firstBounce",
                story.firstBounce()
                        .<JsonElement>map(bounce -> {
                            JsonObject first = new JsonObject();
                            first.addProperty("t", round(bounce.t().value(), 5));
                            first.addProperty("range", round(bounce.range(), 3));
                            return first;
                        })
                        .orElse(JsonNull.INSTANCE));
        int into = 0;
        for (Shot.Crossed crossed : story.crossings()) {
            into += crossed.crossing().into() ? 1 : 0;
        }
        summary.addProperty("crossings", into);
        return summary;
    }

    private static JsonObject shotJson(int id, long at, ShotSettings settings, Shot.Finished finished, double spent) {
        Shot.Story story = finished.story();
        WheelPair.Launched launch = story.launched();
        JsonObject shot = new JsonObject();
        shot.addProperty("id", id);
        shot.addProperty("at", at);
        JsonObject pollen = new JsonObject();
        pollen.addProperty("radius", story.pollen().radius());
        pollen.addProperty("colour", POLLEN_COLOUR);
        shot.add("pollen", pollen);

        JsonArray listed = new JsonArray();
        for (ShotSetting setting : ShotSetting.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", setting.asked());
            one.addProperty("group", setting.group().asked());
            one.addProperty("label", setting.label());
            one.addProperty("unit", setting.unit());
            one.addProperty("value", settings.value(setting));
            one.addProperty("byDefault", setting.byDefault());
            listed.add(one);
        }
        shot.add("settings", listed);

        JsonObject launched = new JsonObject();
        launched.add("at", vector(launch.at(), 4));
        launched.add("velocity", vector(launch.velocity(), 4));
        launched.add("spin", vector(launch.spin(), 4));
        launched.addProperty("speed", round(launch.velocity().length(), 4));
        launched.addProperty("spinRpm", round(launch.spinRpm(), 2));
        launched.addProperty("spinKind", launch.kind().name().toLowerCase(Locale.ROOT));
        launched.addProperty(
                "spinParameter",
                round(story.world().air().spinParameter(story.pollen(), launch.velocity(), launch.spin()), 5));
        launched.add("surfaces", pair(round(launch.surfaceOne(), 4), round(launch.surfaceTwo(), 4)));
        launched.add("after", pair(round(launch.surfaceOneAfter(), 4), round(launch.surfaceTwoAfter(), 4)));
        shot.add("launch", launched);

        JsonObject air = new JsonObject();
        air.addProperty("density", round(story.world().air().density(), 5));
        shot.add("air", air);

        JsonObject tilts = new JsonObject();
        for (Field.Hive hive : FIELD.hives()) {
            tilts.addProperty(hive.alliance(), round(story.world().colliders().tilt(hive.alliance()), 5));
        }
        shot.add("tilts", tilts);
        shot.add("colliders", colliders(story));

        JsonArray frames = new JsonArray();
        for (Shot.Frame frame : story.frames()) {
            Shot.Moving moving = frame.pollen();
            JsonArray row = new JsonArray();
            row.add(round(frame.t().value(), 5));
            for (JsonElement x : vector(moving.at(), 4)) {
                row.add(x);
            }
            for (JsonElement v : vector(moving.velocity(), 4)) {
                row.add(v);
            }
            for (JsonElement w : vector(moving.spin(), 4)) {
                row.add(w);
            }
            for (JsonElement q : turn(moving.turn())) {
                row.add(q);
            }
            row.add(PHASES[moving.phase().ordinal()]);
            frames.add(row);
        }
        shot.add("frames", frames);
        shot.add("pieces", pieces(story));

        JsonArray impacts = new JsonArray();
        for (Shot.Impact impact : story.impacts()) {
            JsonObject one = new JsonObject();
            boolean mine = impact.hit().ball().equals(Shot.LAUNCHED);
            one.addProperty("t", round(impact.t().value(), 5));
            one.add("at", vector(impact.at(), 4));
            one.add("normal", vector(impact.normal(), 5));
            one.addProperty(
                    "against", mine ? impact.hit().against() : impact.hit().ball());
            one.addProperty("surface", impact.hit().surface().asked());
            one.addProperty("approach", round(impact.hit().approach(), 4));
            one.addProperty("slip", round(impact.hit().slip(), 4));
            one.addProperty("restitution", round(impact.hit().restitution(), 5));
            one.addProperty("gripped", impact.hit().gripped());
            one.add("before", motion(impact.velocityBefore(), impact.spinBefore()));
            one.add("after", motion(impact.velocityAfter(), impact.spinAfter()));
            impacts.add(one);
        }
        shot.add("impacts", impacts);

        JsonArray crossings = new JsonArray();
        for (Shot.Crossed crossed : story.crossings()) {
            JsonObject one = new JsonObject();
            one.addProperty("t", round(crossed.t().value(), 5));
            one.addProperty("cell", crossed.crossing().cell());
            one.addProperty("alliance", crossed.crossing().alliance());
            one.addProperty("upturned", crossed.crossing().upturned());
            one.addProperty("into", crossed.crossing().into());
            one.add("at", vector(crossed.crossing().at(), 4));
            one.addProperty("offset", round(crossed.crossing().offset(), 4));
            one.addProperty("rim", round(crossed.crossing().rim(), 4));
            one.addProperty("angle", round(crossed.crossing().angleDegrees(), 3));
            one.addProperty("speed", round(crossed.speed(), 4));
            crossings.add(one);
        }
        shot.add("crossings", crossings);

        JsonObject apex = new JsonObject();
        apex.addProperty("t", round(story.apex().t().value(), 5));
        apex.add("at", vector(story.apex().pollen().at(), 4));
        shot.add("apex", apex);
        shot.add(
                "firstBounce",
                story.firstBounce()
                        .<JsonElement>map(bounce -> {
                            JsonObject first = new JsonObject();
                            first.addProperty("t", round(bounce.t().value(), 5));
                            first.addProperty("range", round(bounce.range(), 4));
                            first.add("at", vector(bounce.at(), 4));
                            return first;
                        })
                        .orElse(JsonNull.INSTANCE));
        shot.add("ending", ending(finished.finish()));

        double simulated = story.simulated().value();
        JsonObject computed = new JsonObject();
        computed.addProperty("seconds", spent);
        computed.addProperty("simulated", round(simulated, 5));
        computed.addProperty("realTime", simulated > 0 ? spent / simulated : 0);
        computed.addProperty("work", story.work());
        shot.add("computed", computed);
        return shot;
    }

    private static JsonObject motion(Vec3 velocity, Vec3 spin) {
        JsonObject motion = new JsonObject();
        motion.add("velocity", vector(velocity, 4));
        motion.add("spin", vector(spin, 4));
        return motion;
    }

    private static JsonArray turn(Turn turn) {
        JsonArray q = new JsonArray();
        q.add(round(turn.w(), 5));
        q.add(round(turn.x(), 5));
        q.add(round(turn.y(), 5));
        q.add(round(turn.z(), 5));
        return q;
    }

    private static JsonArray colliders(Shot.Story story) {
        JsonArray colliders = new JsonArray();
        for (Collider collider : story.world().colliders().all()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", collider.name());
            one.addProperty("surface", collider.surface().asked());
            one.addProperty("kind", collider.kind().name().toLowerCase(Locale.ROOT));
            JsonArray faces = new JsonArray();
            for (Ring<Vec3> face : collider.faces()) {
                JsonArray corners = new JsonArray();
                for (Vec3 corner : face.all()) {
                    corners.add(vector(corner, 3));
                }
                faces.add(corners);
            }
            one.add("faces", faces);
            colliders.add(one);
        }
        return colliders;
    }

    private static String colourOf(World.Piece piece) {
        if (piece.kind() == Field.Kind.POLLEN) {
            return POLLEN_COLOUR;
        }
        for (Field.Hive hive : FIELD.hives()) {
            if (piece.alliance().map(hive.alliance()::equals).orElse(false)) {
                return hive.colour();
            }
        }
        return POLLEN_COLOUR;
    }

    private static JsonArray pieces(Shot.Story story) {
        JsonArray pieces = new JsonArray();
        for (Shot.Track track : story.pieces()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", track.piece().name());
            one.addProperty("kind", track.piece().kind().named());
            one.addProperty("colour", colourOf(track.piece()));
            one.addProperty("radius", track.piece().shape().radius());
            one.addProperty("fixed", track.piece().fixed());
            one.add("start", vector(track.piece().at(), 4));
            JsonArray samples = new JsonArray();
            for (Shot.Placed placed : track.samples()) {
                JsonArray row = new JsonArray();
                row.add(round(placed.t().value(), 5));
                for (JsonElement x : vector(placed.at(), 4)) {
                    row.add(x);
                }
                for (JsonElement q : turn(placed.turn())) {
                    row.add(q);
                }
                samples.add(row);
            }
            one.add("track", samples);
            if (track.fate() instanceof Shot.Fate.Gone gone) {
                one.addProperty("gone", round(gone.t().value(), 5));
            } else {
                one.add("gone", JsonNull.INSTANCE);
            }
            pieces.add(one);
        }
        return pieces;
    }

    private static String csv(Shot.Finished finished) {
        StringBuilder csv = new StringBuilder("t,x,y,z,vx,vy,vz,wx,wy,wz,phase\n");
        for (Shot.Frame frame : finished.story().frames()) {
            Shot.Moving moving = frame.pollen();
            csv.append(round(frame.t().value(), 5));
            for (Vec3 v : List.of(moving.at(), moving.velocity(), moving.spin())) {
                csv.append(',')
                        .append(round(v.x(), 4))
                        .append(',')
                        .append(round(v.y(), 4))
                        .append(',')
                        .append(round(v.z(), 4));
            }
            csv.append(',')
                    .append(moving.phase().name().toLowerCase(Locale.ROOT))
                    .append('\n');
        }
        return csv.toString();
    }
}
