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
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Feed;
import org.firstinspires.ftc.teamcode.simcore.FeedSetting;
import org.firstinspires.ftc.teamcode.simcore.FeedSettings;
import org.firstinspires.ftc.teamcode.simcore.Turn;
import org.firstinspires.ftc.teamcode.simcore.Vec3;

final class SimFeeds {
    static final int MOST_KEPT = 30;
    static final long MOST_KEPT_BYTES = 24L * 1024 * 1024;
    static final String SETTINGS_KEPT = "settings";
    static final String FED_SO_FAR = "fed";
    static final String REMEMBERED_FILE = "feed-settings.json";
    static final int CHUNK_FRAMES = 250;

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private record Kept(int id, JsonObject summary, String feed, List<String> chunks) {}

    private sealed interface Asked permits Asked.Settled, Asked.Refused {
        record Settled(FeedSettings settings) implements Asked {}

        record Refused(JsonObject refusal) implements Asked {}

        static Asked refused(String setting, String rule) {
            JsonObject refusal = new JsonObject();
            refusal.add("setting", setting == null ? JsonNull.INSTANCE : new JsonPrimitive(setting));
            refusal.addProperty("rule", rule);
            return new Refused(refusal);
        }

        static Asked unknown(String setting, String rule) {
            JsonObject refusal = new JsonObject();
            refusal.addProperty("setting", setting);
            refusal.addProperty("rule", rule);
            refusal.addProperty("unknown", setting);
            return new Refused(refusal);
        }
    }

    private final Path file;
    private final Store store;
    private final Clock clock;
    private final AtomicBoolean simulating = new AtomicBoolean();
    private final List<Kept> feeds = new ArrayList<>();
    private int fed;
    private FeedSettings remembered = FeedSettings.defaults();
    private String forgot = "";

    SimFeeds(Path file, Store store, Clock clock) {
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
            forgot = "The feed settings kept in " + file + " could not be read (" + e.getMessage()
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
            forgot = "The feed settings kept in " + file + " are not settings (" + e.getMessage()
                    + "), so the form starts from the defaults.";
            return;
        }
        if (kept == null) {
            return;
        }
        Optional<Double> count = numberIn(kept.get(FED_SO_FAR));
        if (count.isPresent() && count.get() >= 0 && count.get() < Integer.MAX_VALUE) {
            fed = (int) Math.floor(count.get());
        }
        JsonElement settingsKept = kept.get(SETTINGS_KEPT);
        JsonObject asKept =
                settingsKept != null && settingsKept.isJsonObject() ? settingsKept.getAsJsonObject() : new JsonObject();
        Map<FeedSetting, Double> set = new EnumMap<>(FeedSetting.class);
        List<String> dropped = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : asKept.entrySet()) {
            Checked<FeedSetting> named = FeedSetting.named(entry.getKey());
            if (!(named instanceof Checked.Ok<FeedSetting> ok)) {
                dropped.add(entry.getKey() + " (there is no such setting now)");
                continue;
            }
            Optional<Double> number = numberIn(entry.getValue());
            if (number.isEmpty()) {
                dropped.add(entry.getKey() + " (kept as " + entry.getValue() + ", which is not a number)");
                continue;
            }
            set.put(ok.value(), number.get());
        }
        Checked<FeedSettings> settings = FeedSettings.of(set);
        if (settings instanceof Checked.Ok<FeedSettings> ok) {
            remembered = ok.value();
        } else {
            dropped.add("every setting (" + rule(settings) + ")");
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
        FeedSettings now;
        String said;
        synchronized (this) {
            now = remembered;
            said = forgot;
        }
        JsonArray groups = new JsonArray();
        for (FeedSetting.Group group : FeedSetting.Group.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", group.asked());
            one.addProperty("label", group.label());
            one.addProperty("says", group.says());
            groups.add(one);
        }
        JsonArray listed = new JsonArray();
        for (FeedSetting setting : FeedSetting.values()) {
            JsonObject one = described(setting, now);
            one.addProperty("says", setting.says());
            one.addProperty("least", setting.least());
            one.addProperty("most", setting.most());
            one.addProperty("whole", setting.whole());
            listed.add(one);
        }
        JsonObject listing = new JsonObject();
        listing.add("groups", groups);
        listing.add("settings", listed);
        listing.addProperty("forgot", said);
        return Response.json(GSON.toJson(listing));
    }

    private static JsonObject described(FeedSetting setting, FeedSettings settings) {
        JsonObject one = new JsonObject();
        one.addProperty("name", setting.asked());
        one.addProperty("group", setting.group().asked());
        one.addProperty("label", setting.label());
        one.addProperty("unit", setting.unit());
        one.addProperty("byDefault", setting.byDefault());
        one.addProperty("value", settings.value(setting));
        return one;
    }

    private static Asked asked(String body) {
        JsonObject json;
        try {
            json = GSON.fromJson(body == null || body.isBlank() ? "{}" : body, JsonObject.class);
        } catch (RuntimeException e) {
            return Asked.refused(
                    null, "a feed is asked for with an object of numbers by setting, as {\"pollen\": 1}, not " + body);
        }
        if (json == null) {
            return new Asked.Settled(FeedSettings.defaults());
        }
        Map<FeedSetting, Double> set = new EnumMap<>(FeedSetting.class);
        for (Map.Entry<String, JsonElement> entry : json.entrySet()) {
            Checked<FeedSetting> named = FeedSetting.named(entry.getKey());
            if (!(named instanceof Checked.Ok<FeedSetting> ok)) {
                return Asked.unknown(entry.getKey(), rule(named));
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
        Checked<FeedSettings> settings = FeedSettings.of(set);
        if (!(settings instanceof Checked.Ok<FeedSettings> ok)) {
            return Asked.refused(null, rule(settings));
        }
        return new Asked.Settled(ok.value());
    }

    private static Response refusedAs(Asked asked) {
        return Response.json(
                400, GSON.toJson(asked instanceof Asked.Refused refused ? refused.refusal() : new JsonObject()));
    }

    Response feed(String body) {
        Asked asked = asked(body);
        if (!(asked instanceof Asked.Settled settled)) {
            return refusedAs(asked);
        }
        FeedSettings settings = settled.settings();
        if (!simulating.compareAndSet(false, true)) {
            return Response.error(409, "a feed is already being simulated; it takes a moment");
        }
        try {
            long began = clock.nanos();
            Checked<Feed.Fed> run = settings.plan().then(Feed.Plan::run);
            if (!(run instanceof Checked.Ok<Feed.Fed> ok)) {
                return refusedAs(Asked.refused(null, rule(run)));
            }
            double spent = (clock.nanos() - began) / 1e9;
            return Response.json(GSON.toJson(kept(settings, ok.value(), spent)));
        } finally {
            simulating.set(false);
        }
    }

    private synchronized JsonObject kept(FeedSettings settings, Feed.Fed run, double spent) {
        int id = ++fed;
        long at = clock.millisSinceEpoch();
        JsonObject summary = summary(id, at, settings, run, spent);
        List<String> chunks = chunks(run.frames());
        JsonObject feed = new JsonObject();
        feed.addProperty("id", id);
        feed.add("summary", summary);
        feed.add("settings", settingsOf(settings));
        feed.add("events", events(run.events()));
        feed.addProperty("frames", run.frames().size());
        feed.addProperty("chunks", chunks.size());
        feed.addProperty("pollenRadius", Feed.POLLEN_RADIUS_IN);
        feeds.add(new Kept(id, summary, GSON.toJson(feed), chunks));
        while (feeds.size() > MOST_KEPT || (feeds.size() > 1 && keptBytes() > MOST_KEPT_BYTES)) {
            feeds.remove(0);
        }
        remembered = settings;
        forgot = "";
        JsonObject kept = new JsonObject();
        kept.add(SETTINGS_KEPT, changed(settings));
        kept.addProperty(FED_SO_FAR, fed);
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
        for (Kept kept : feeds) {
            bytes += kept.feed().length();
            for (String chunk : kept.chunks()) {
                bytes += chunk.length();
            }
        }
        return bytes;
    }

    synchronized Response listing() {
        JsonArray listed = new JsonArray();
        for (int i = feeds.size() - 1; i >= 0; i--) {
            listed.add(feeds.get(i).summary());
        }
        JsonObject listing = new JsonObject();
        listing.add("feeds", listed);
        return Response.json(GSON.toJson(listing));
    }

    private synchronized Optional<Kept> find(String id) {
        if (id == null || !id.matches("[0-9]{1,9}")) {
            return Optional.empty();
        }
        for (Kept kept : feeds) {
            if (String.valueOf(kept.id()).equals(id)) {
                return Optional.of(kept);
            }
        }
        return Optional.empty();
    }

    Response fed(String id) {
        return find(id).map(kept -> Response.json(kept.feed())).orElse(Response.error(404, "no such feed: " + id));
    }

    Response frames(String id, String chunk) {
        Optional<Kept> kept = find(id);
        if (kept.isEmpty()) {
            return Response.error(404, "no such feed: " + id);
        }
        List<String> chunks = kept.get().chunks();
        if (chunk == null || !chunk.matches("[0-9]{1,9}") || Integer.parseInt(chunk) >= chunks.size()) {
            return Response.error(404, "feed " + id + " has frames in chunks 0 to " + (chunks.size() - 1));
        }
        return Response.json(chunks.get(Integer.parseInt(chunk)));
    }

    private static List<String> chunks(List<Feed.Frame> frames) {
        List<String> chunks = new ArrayList<>();
        int count = Math.max(1, (frames.size() + CHUNK_FRAMES - 1) / CHUNK_FRAMES);
        for (int chunk = 0; chunk < count; chunk++) {
            JsonArray rows = new JsonArray();
            for (Feed.Frame frame :
                    frames.subList(chunk * CHUNK_FRAMES, Math.min(frames.size(), (chunk + 1) * CHUNK_FRAMES))) {
                rows.add(frame(frame));
            }
            JsonObject one = new JsonObject();
            one.addProperty("chunk", chunk);
            one.addProperty("chunks", count);
            one.add("frames", rows);
            chunks.add(GSON.toJson(one));
        }
        return chunks;
    }

    private static JsonObject frame(Feed.Frame frame) {
        JsonObject one = new JsonObject();
        one.addProperty("t", round(frame.t().value(), 5));
        JsonArray spinner = new JsonArray();
        spinner.add(round(frame.spinnerDegrees(), 3));
        spinner.add(round(frame.spinnerRevolutionsPerSecond(), 4));
        spinner.add(round(frame.spinnerLoadNewtonMetres(), 4));
        one.add("spinner", spinner);
        JsonArray ramp = new JsonArray();
        ramp.add(round(frame.rampDegrees(), 3));
        ramp.add(round(frame.rampTorqueNewtonMetres(), 4));
        one.add("ramp", ramp);
        JsonArray pollen = new JsonArray();
        for (Feed.Seen seen : frame.pollen()) {
            JsonObject each = new JsonObject();
            each.addProperty("name", seen.name());
            each.add("at", vector(seen.at(), 4));
            each.add("turn", turn(seen.turn()));
            pollen.add(each);
        }
        one.add("pollen", pollen);
        return one;
    }

    private static JsonObject summary(int id, long at, FeedSettings settings, Feed.Fed run, double spent) {
        double simulated = run.simulated().value();
        JsonArray thrownAt = new JsonArray();
        for (Feed.Event event : run.events()) {
            if (event instanceof Feed.Event.Thrown) {
                thrownAt.add(round(event.at().value(), 5));
            }
        }
        JsonObject summary = new JsonObject();
        summary.addProperty("id", id);
        summary.addProperty("at", at);
        summary.add("changed", changed(settings));
        summary.addProperty("thrown", thrownAt.size());
        summary.add("thrownAt", thrownAt);
        summary.add("ending", ending(run.finish()));
        summary.addProperty("seconds", round(simulated, 5));
        summary.addProperty("computeSeconds", spent);
        summary.addProperty("realTime", simulated > 0 ? spent / simulated : 0);
        summary.addProperty("work", run.work());
        return summary;
    }

    private static JsonObject ending(Feed.Finish finish) {
        JsonObject ending = new JsonObject();
        String kind;
        String says;
        if (finish instanceof Feed.Finish.AllLeft) {
            kind = "all left";
            says = "every pollen has left the feeder";
        } else if (finish instanceof Feed.Finish.AtRest) {
            kind = "at rest";
            says = "the spinner and ramp are still and every pollen has come to rest";
        } else if (finish instanceof Feed.Finish.TooMuchWork) {
            kind = "too much work";
            says = "the feed took more work than its budget, so it was stopped";
        } else {
            kind = "out of time";
            says = "the run's length has passed";
        }
        ending.addProperty("kind", kind);
        ending.addProperty("says", says);
        ending.addProperty("t", round(finish.at().value(), 5));
        return ending;
    }

    private static JsonArray events(List<Feed.Event> events) {
        JsonArray listed = new JsonArray();
        for (Feed.Event event : events) {
            JsonObject one = new JsonObject();
            one.addProperty("t", round(event.at().value(), 5));
            if (event instanceof Feed.Event.Thrown thrown) {
                one.addProperty("kind", "thrown");
                one.addProperty("pollen", thrown.pollen());
                one.add("at", vector(thrown.from(), 4));
                one.add("velocity", vector(thrown.velocity(), 4));
                one.add("spin", vector(thrown.spin(), 4));
                one.addProperty("speed", round(thrown.velocity().length(), 3));
                one.addProperty(
                        "angle",
                        round(
                                Math.toDegrees(Math.atan2(
                                        thrown.velocity().z(),
                                        Math.hypot(
                                                thrown.velocity().x(),
                                                thrown.velocity().y()))),
                                2));
            } else if (event instanceof Feed.Event.FellOut out) {
                one.addProperty("kind", "fell out");
                one.addProperty("pollen", out.pollen());
                one.add("at", vector(out.from(), 4));
            } else if (event instanceof Feed.Event.Jammed jammed) {
                one.addProperty("kind", "jammed");
                JsonArray pinned = new JsonArray();
                for (String pollen : jammed.pollen()) {
                    pinned.add(pollen);
                }
                one.add("pollen", pinned);
            }
            listed.add(one);
        }
        return listed;
    }

    private static JsonArray settingsOf(FeedSettings settings) {
        JsonArray listed = new JsonArray();
        for (FeedSetting setting : FeedSetting.values()) {
            listed.add(described(setting, settings));
        }
        return listed;
    }

    private static JsonObject changed(FeedSettings settings) {
        JsonObject changed = new JsonObject();
        for (Map.Entry<FeedSetting, Double> entry : settings.changed().entrySet()) {
            changed.addProperty(entry.getKey().asked(), entry.getValue());
        }
        return changed;
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

    private static JsonArray turn(Turn turn) {
        JsonArray q = new JsonArray();
        q.add(round(turn.w(), 5));
        q.add(round(turn.x(), 5));
        q.add(round(turn.y(), 5));
        q.add(round(turn.z(), 5));
        return q;
    }
}
