package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.Facing;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.MotorType;
import org.firstinspires.ftc.teamcode.simcore.Overrides;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;

final class SimOverrides {
    static final String FILE = "robot-overrides.json";

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final String CONSTANTS = "constants";
    private static final String HUB = "hub";
    private static final String CONFIGURATION = "configuration";
    private static final String DIRECTIONS = "directions";
    private static final String LOGO = "logo";
    private static final String USB = "usb";
    private static final String NAME = "name";
    private static final String TYPE = "type";
    private static final String ENCODER = "encoder";
    private static final String EVERY_EDIT = "every edit";

    private final Path file;
    private final Store store;
    private Overrides overrides;
    private List<Overrides.Notice> notices;

    SimOverrides(Path file, Store store, RobotDescription defaults) {
        this.file = file;
        this.store = store;
        Overrides.Recalled recalled = recall(file, store, defaults);
        this.overrides = recalled.overrides();
        this.notices = recalled.notices();
    }

    private static Overrides.Recalled recall(Path file, Store store, RobotDescription defaults) {
        Optional<byte[]> bytes;
        try {
            bytes = store.readIfThere(file);
        } catch (RuntimeException e) {
            return unread(defaults, "could not read " + file + " (" + e.getMessage() + ")");
        }
        if (bytes.isEmpty()) {
            return Overrides.recalled(defaults, List.of());
        }
        JsonElement json;
        try {
            json = GSON.fromJson(new String(bytes.get(), StandardCharsets.UTF_8), JsonElement.class);
        } catch (JsonParseException e) {
            return unread(defaults, file + " is not JSON (" + e.getMessage() + ")");
        }
        if (json == null || !json.isJsonObject()) {
            return unread(defaults, file + " is not an object of edits but " + json);
        }
        return Overrides.recalled(defaults, kept(json.getAsJsonObject()));
    }

    private static Overrides.Recalled unread(RobotDescription defaults, String why) {
        return new Overrides.Recalled(
                Overrides.none(defaults),
                List.of(new Overrides.Notice(EVERY_EDIT, Overrides.Fate.BACK_TO_DEFAULT, why)));
    }

    synchronized RobotDescription description() {
        return overrides.description();
    }

    Response listing() {
        Overrides now;
        List<Overrides.Notice> said;
        synchronized (this) {
            now = overrides;
            said = notices;
        }
        return Response.json(GSON.toJson(listingOf(now, said)));
    }

    Response save(String body) {
        JsonElement json;
        try {
            json = GSON.fromJson(body == null || body.isBlank() ? "{}" : body, JsonElement.class);
        } catch (JsonParseException e) {
            return refused(
                    "the robot's edits are an object of constants, hub, configuration and directions, not " + body);
        }
        if (json == null || !json.isJsonObject()) {
            return refused(
                    "the robot's edits are an object of constants, hub, configuration and directions, not " + json);
        }
        Overrides current;
        synchronized (this) {
            current = overrides;
        }
        Checked<Overrides> asked = Overrides.asked(current.defaults(), kept(json.getAsJsonObject()));
        if (asked instanceof Checked.Rejected<Overrides> rejected) {
            return refused(rejected.rule());
        }
        Overrides taken = Valid.value(asked);
        synchronized (this) {
            try {
                store.writeWhole(file, GSON.toJson(editsJson(taken)).getBytes(StandardCharsets.UTF_8));
            } catch (RuntimeException e) {
                return Response.json(
                        500, GSON.toJson(rule("could not save the robot's edits in " + file + ": " + e.getMessage())));
            }
            overrides = taken;
            notices = List.of();
        }
        return listing();
    }

    private static Response refused(String rule) {
        return Response.json(400, GSON.toJson(rule(rule)));
    }

    private static JsonObject rule(String rule) {
        JsonObject refusal = new JsonObject();
        refusal.addProperty("rule", rule);
        return refusal;
    }

    static List<Overrides.Kept> kept(JsonObject json) {
        List<Overrides.Kept> kept = new ArrayList<>();
        for (Map.Entry<String, JsonElement> part : json.entrySet()) {
            String key = part.getKey();
            JsonElement value = part.getValue();
            switch (key) {
                case CONSTANTS -> kept.addAll(constants(value));
                case HUB -> kept.addAll(hub(value));
                case CONFIGURATION -> kept.addAll(configuration(value));
                case DIRECTIONS -> kept.addAll(directions(value));
                default -> kept.add(new Overrides.Kept.Gone(key));
            }
        }
        return kept;
    }

    private static List<Overrides.Kept> constants(JsonElement json) {
        if (!json.isJsonObject()) {
            return List.of(new Overrides.Kept.Unreadable(
                    CONSTANTS, "the constants are an object of numbers by name, not " + json));
        }
        List<Overrides.Kept> kept = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            Checked<Constant> named = Constant.named(entry.getKey());
            if (!(named instanceof Checked.Ok<Constant> ok)) {
                kept.add(new Overrides.Kept.Gone(entry.getKey()));
                continue;
            }
            Constant constant = ok.value();
            JsonElement value = entry.getValue();
            if (!(value.isJsonPrimitive() && value.getAsJsonPrimitive().isNumber())) {
                kept.add(new Overrides.Kept.Unreadable(
                        constant.label(), constant.label() + " is a number of " + constant.unit() + ", not " + value));
                continue;
            }
            kept.add(new Overrides.Kept.Known(new Overrides.Edit.OfConstant(constant, value.getAsDouble())));
        }
        return kept;
    }

    private static List<Overrides.Kept> hub(JsonElement json) {
        if (!json.isJsonObject()) {
            return List.of(new Overrides.Kept.Unreadable(HUB, "the hub is an object of logo and usb, not " + json));
        }
        List<Overrides.Kept> kept = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            Overrides.HubFace face;
            switch (entry.getKey()) {
                case LOGO -> face = Overrides.HubFace.LOGO;
                case USB -> face = Overrides.HubFace.USB;
                default -> {
                    kept.add(new Overrides.Kept.Gone(HUB + "." + entry.getKey()));
                    continue;
                }
            }
            kept.add(read(
                    face.label(), entry.getValue(), Facing::named, facing -> new Overrides.Edit.OfHub(face, facing)));
        }
        return kept;
    }

    private static List<Overrides.Kept> directions(JsonElement json) {
        if (!json.isJsonObject()) {
            return List.of(new Overrides.Kept.Unreadable(
                    DIRECTIONS, "the directions are an object of mechanisms by name, not " + json));
        }
        List<Overrides.Kept> kept = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            Checked<Mechanism> named = Mechanism.named(entry.getKey());
            if (!(named instanceof Checked.Ok<Mechanism> ok)) {
                kept.add(new Overrides.Kept.Gone(DIRECTIONS + "." + entry.getKey()));
                continue;
            }
            Mechanism mechanism = ok.value();
            kept.add(read(
                    mechanism.label() + "'s direction",
                    entry.getValue(),
                    mechanism::reversedIf,
                    reversed -> new Overrides.Edit.Reversed(mechanism, reversed)));
        }
        return kept;
    }

    private static List<Overrides.Kept> configuration(JsonElement json) {
        if (!json.isJsonObject()) {
            return List.of(new Overrides.Kept.Unreadable(
                    CONFIGURATION, "the configuration's edits are an object of devices by name, not " + json));
        }
        List<Overrides.Kept> kept = new ArrayList<>();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            String device = entry.getKey();
            if (!entry.getValue().isJsonObject()) {
                kept.add(new Overrides.Kept.Unreadable(
                        device, device + "'s edits are an object of name, type and encoder, not " + entry.getValue()));
                continue;
            }
            for (Map.Entry<String, JsonElement> field :
                    entry.getValue().getAsJsonObject().entrySet()) {
                JsonElement value = field.getValue();
                switch (field.getKey()) {
                    case NAME ->
                        kept.add(read(
                                device + "'s name",
                                value,
                                Checked::ok,
                                name -> new Overrides.Edit.Renamed(device, name)));
                    case TYPE ->
                        kept.add(read(
                                device + "'s motor type",
                                value,
                                MotorType::named,
                                type -> new Overrides.Edit.Typed(device, type)));
                    case ENCODER ->
                        kept.add(read(
                                device + "'s encoder port",
                                value,
                                EncoderReads::named,
                                encoder -> new Overrides.Edit.Wired(device, encoder)));
                    default -> kept.add(new Overrides.Kept.Gone(device + "." + field.getKey()));
                }
            }
        }
        return kept;
    }

    private static <T> Overrides.Kept read(
            String what, JsonElement value, Function<String, Checked<T>> named, Function<T, Overrides.Edit> edit) {
        if (!(value.isJsonPrimitive() && value.getAsJsonPrimitive().isString())) {
            return new Overrides.Kept.Unreadable(what, what + " is text, not " + value);
        }
        return named.apply(value.getAsString())
                .fold(
                        read -> new Overrides.Kept.Known(edit.apply(read)),
                        rule -> new Overrides.Kept.Unreadable(what, rule));
    }

    static JsonObject editsJson(Overrides overrides) {
        JsonObject constants = new JsonObject();
        JsonObject hub = new JsonObject();
        JsonObject configuration = new JsonObject();
        JsonObject directions = new JsonObject();
        for (Overrides.Edit edit : overrides.edits()) {
            if (edit instanceof Overrides.Edit.Reversed reversed) {
                directions.addProperty(
                        reversed.mechanism().asked(), reversed.mechanism().way(reversed.reversed()));
            }
            if (edit instanceof Overrides.Edit.OfConstant set) {
                constants.addProperty(set.constant().asked(), set.to());
            }
            if (edit instanceof Overrides.Edit.OfHub face) {
                hub.addProperty(
                        face.face() == Overrides.HubFace.LOGO ? LOGO : USB,
                        face.facing().name());
            }
            if (edit instanceof Overrides.Edit.Renamed renamed) {
                deviceIn(configuration, renamed.device()).addProperty(NAME, renamed.name());
            }
            if (edit instanceof Overrides.Edit.Typed typed) {
                deviceIn(configuration, typed.device())
                        .addProperty(TYPE, typed.type().label());
            }
            if (edit instanceof Overrides.Edit.Wired wired) {
                deviceIn(configuration, wired.device())
                        .addProperty(ENCODER, wired.encoder().asked());
            }
        }
        JsonObject edits = new JsonObject();
        if (!constants.entrySet().isEmpty()) {
            edits.add(CONSTANTS, constants);
        }
        if (!hub.entrySet().isEmpty()) {
            edits.add(HUB, hub);
        }
        if (!configuration.entrySet().isEmpty()) {
            edits.add(CONFIGURATION, configuration);
        }
        if (!directions.entrySet().isEmpty()) {
            edits.add(DIRECTIONS, directions);
        }
        return edits;
    }

    private static JsonObject deviceIn(JsonObject configuration, String device) {
        if (!configuration.has(device)) {
            configuration.add(device, new JsonObject());
        }
        return configuration.getAsJsonObject(device);
    }

    private static JsonObject listingOf(Overrides overrides, List<Overrides.Notice> notices) {
        RobotDescription defaults = overrides.defaults();
        RobotDescription now = overrides.description();
        JsonObject listing = new JsonObject();
        JsonArray devices = new JsonArray();
        List<Configuration.Device> edited = now.configuration().devices();
        int at = 0;
        for (Configuration.Device byDefault : defaults.configuration().devices()) {
            devices.add(deviceJson(byDefault, edited.get(at)));
            at++;
        }
        listing.add(CONFIGURATION, devices);
        JsonArray types = new JsonArray();
        for (MotorType type : MotorType.values()) {
            types.add(type.label());
        }
        listing.add("types", types);
        JsonArray encoders = new JsonArray();
        for (EncoderReads reads : EncoderReads.values()) {
            JsonObject one = new JsonObject();
            one.addProperty(NAME, reads.asked());
            one.addProperty("label", reads.toString());
            encoders.add(one);
        }
        listing.add("encoders", encoders);
        JsonObject hub = new JsonObject();
        hub.addProperty(LOGO, now.hub().logo().name());
        hub.addProperty(USB, now.hub().usb().name());
        JsonObject hubByDefault = new JsonObject();
        hubByDefault.addProperty(LOGO, defaults.hub().logo().name());
        hubByDefault.addProperty(USB, defaults.hub().usb().name());
        hub.add("byDefault", hubByDefault);
        JsonArray faces = new JsonArray();
        for (Facing facing : Facing.values()) {
            faces.add(facing.name());
        }
        hub.add("faces", faces);
        listing.add(HUB, hub);
        JsonArray directions = new JsonArray();
        for (Mechanism mechanism : Mechanism.values()) {
            JsonObject one = new JsonObject();
            one.addProperty(NAME, mechanism.asked());
            one.addProperty("label", mechanism.label());
            JsonArray ways = new JsonArray();
            for (boolean reversed : new boolean[] {false, true}) {
                JsonObject way = new JsonObject();
                way.addProperty(NAME, mechanism.way(reversed));
                way.addProperty("says", mechanism.says(reversed));
                ways.add(way);
            }
            one.add("ways", ways);
            one.addProperty("byDefault", mechanism.way(defaults.directions().reversed(mechanism)));
            one.addProperty("value", mechanism.way(now.directions().reversed(mechanism)));
            directions.add(one);
        }
        listing.add(DIRECTIONS, directions);
        JsonArray groups = new JsonArray();
        for (Constant.Group group : Constant.Group.values()) {
            JsonObject one = new JsonObject();
            one.addProperty(NAME, group.asked());
            one.addProperty("label", group.label());
            one.addProperty("says", group.says());
            groups.add(one);
        }
        listing.add("groups", groups);
        JsonArray constants = new JsonArray();
        for (Constant constant : Constant.values()) {
            JsonObject one = new JsonObject();
            one.addProperty(NAME, constant.asked());
            one.addProperty("group", constant.group().asked());
            one.addProperty("label", constant.label());
            one.addProperty("unit", constant.unit());
            one.addProperty("says", constant.says());
            one.addProperty("least", constant.least());
            one.addProperty("most", constant.most());
            one.addProperty("byDefault", defaults.constants().value(constant));
            one.addProperty("value", now.constants().value(constant));
            constants.add(one);
        }
        listing.add(CONSTANTS, constants);
        JsonArray said = new JsonArray();
        for (Overrides.Notice notice : notices) {
            said.add(notice.said());
        }
        listing.add("notices", said);
        listing.add("edits", editsJson(overrides));
        listing.addProperty("described", SimDescription.described(now));
        return listing;
    }

    private static JsonObject deviceJson(Configuration.Device byDefault, Configuration.Device now) {
        JsonObject device = new JsonObject();
        device.addProperty("device", byDefault.name());
        device.addProperty("kind", byDefault.kind());
        device.add("value", fieldsOf(now));
        device.add("byDefault", fieldsOf(byDefault));
        return device;
    }

    private static JsonObject fieldsOf(Configuration.Device device) {
        JsonObject fields = new JsonObject();
        fields.addProperty(NAME, device.name());
        if (device instanceof Configuration.Motor motor) {
            fields.addProperty(TYPE, motor.type().label());
            fields.addProperty(ENCODER, motor.encoder().asked());
        }
        return fields;
    }
}
