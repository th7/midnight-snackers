package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.Facing;
import org.firstinspires.ftc.teamcode.simcore.HubMounting;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.MotorType;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;

public final class SimDescription {
    public static final String FILE = "robot-description.json";

    static final String CONFIGURATION = "configuration";
    static final String HUB = "hub";
    static final String CONSTANTS = "constants";
    static final String DIRECTIONS = "directions";

    private static final List<String> PARTS = List.of(CONFIGURATION, HUB, CONSTANTS, DIRECTIONS);
    private static final List<String> HUB_FACES = List.of("logo", "usb");
    private static final List<String> MOTOR_FIELDS = List.of("name", "kind", "type", "encoder");
    private static final List<String> OTHER_FIELDS = List.of("name", "kind");

    private static final TypeAdapter<JsonElement> ELEMENT = new Gson().getAdapter(JsonElement.class);

    private static final String MOTOR = "motor";
    private static final String IMU = "imu";
    private static final String CONTINUOUS_SERVO = "continuous_servo";
    private static final String SERVO = "servo";
    private static final String VOLTAGE_SENSOR = "voltage_sensor";
    private static final List<String> KINDS = List.of(MOTOR, IMU, CONTINUOUS_SERVO, SERVO, VOLTAGE_SENSOR);

    private SimDescription() {}

    public static Checked<RobotDescription> read(Path file) {
        String text;
        try {
            text = new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
        } catch (IOException e) {
            return Checked.rejected("could not read " + file + ": " + e);
        }
        JsonElement json;
        try {
            JsonReader reader = new JsonReader(new StringReader(text));
            json = ELEMENT.read(reader);
            if (reader.peek() != JsonToken.END_DOCUMENT) {
                return Checked.rejected(file + " is not JSON: it goes on after its one value");
            }
        } catch (IOException | JsonParseException | IllegalStateException e) {
            return Checked.rejected(file + " is not JSON: " + e.getMessage());
        }
        return whole(json).fold(Checked::ok, rule -> Checked.rejected(file + ": " + rule));
    }

    private static Checked<RobotDescription> whole(JsonElement json) {
        Checked<JsonObject> fields = fields(json, "the robot description", PARTS, PARTS);
        if (fields instanceof Checked.Rejected<JsonObject> rejected) {
            return Checked.rejected(rejected.rule());
        }
        JsonObject parts = Valid.value(fields);
        JsonElement constants = parts.get(CONSTANTS);
        if (constants.isJsonObject()) {
            for (Constant constant : Constant.values()) {
                if (!constants.getAsJsonObject().has(constant.asked())) {
                    return Checked.rejected("the robot description's constants name every simulation constant, and "
                            + constant.asked() + " is missing");
                }
            }
        }
        Checked<Directions> directions = directions(parts.get(DIRECTIONS));
        if (directions instanceof Checked.Rejected<Directions> rejected) {
            return Checked.rejected(rejected.rule());
        }
        for (Mechanism mechanism : Mechanism.values()) {
            if (!parts.getAsJsonObject(DIRECTIONS).has(mechanism.asked())) {
                return Checked.rejected("the robot description's directions name every mechanism, and "
                        + mechanism.asked() + " is missing");
            }
        }
        return configuration(parts.get(CONFIGURATION))
                .then(configuration -> hub(parts.get(HUB))
                        .then(hub -> SimConstants.fromJson(constants)
                                .then(read -> directions.map(
                                        directed -> RobotDescription.of(configuration, hub, read, directed)))));
    }

    public static void addWhatDiffers(RobotDescription description, JsonObject line) {
        if (!description.constants().asBuilt()) {
            line.add(CONSTANTS, SimConstants.changed(description.constants()));
        }
        if (!description.hubAsBuilt()) {
            line.add(HUB, hubJson(description.hub()));
        }
        if (!description.configurationAsBuilt()) {
            line.add(CONFIGURATION, configurationJson(description.configuration()));
        }
        if (!description.directionsAsBuilt()) {
            line.add(DIRECTIONS, directionsJson(description.directions()));
        }
    }

    public static Checked<RobotDescription> fromStartLine(JsonObject line) {
        Checked<Configuration> configuration = line.has(CONFIGURATION)
                ? configuration(line.get(CONFIGURATION))
                : Checked.ok(Configuration.reginalds());
        Checked<HubMounting> hub = line.has(HUB) ? hub(line.get(HUB)) : Checked.ok(HubMounting.reginalds());
        Checked<Directions> directions =
                line.has(DIRECTIONS) ? directions(line.get(DIRECTIONS)) : Checked.ok(Directions.reginalds());
        return configuration.then(configured ->
                hub.then(mounted -> directions.then(directed -> SimConstants.fromJson(line.get(CONSTANTS))
                        .map(constants -> RobotDescription.of(configured, mounted, constants, directed)))));
    }

    public static String described(RobotDescription description) {
        if (description.asBuilt()) {
            return "as built";
        }
        List<String> changes = new ArrayList<>();
        if (!description.constants().asBuilt()) {
            changes.add(SimConstants.described(description.constants()));
        }
        if (!description.hubAsBuilt()) {
            changes.add("hub " + description.hub() + " (built " + HubMounting.reginalds() + ")");
        }
        if (!description.configurationAsBuilt()) {
            changes.add("configuration " + said(description.configuration()) + " (built "
                    + said(Configuration.reginalds()) + ")");
        }
        for (Mechanism mechanism : Mechanism.values()) {
            if (description.directions().reversed(mechanism)) {
                changes.add(mechanism.asked() + " " + spoken(mechanism.way(true)) + " on positive power (built "
                        + spoken(mechanism.way(false)) + ")");
            }
        }
        return String.join("; ", changes);
    }

    private static String spoken(String way) {
        return way.replace('_', ' ');
    }

    static Checked<Directions> directions(JsonElement json) {
        if (json == null || !json.isJsonObject()) {
            return Checked.rejected("the directions are an object of mechanisms by name, not " + json);
        }
        Directions directions = Directions.reginalds();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            Checked<Mechanism> named = Mechanism.named(entry.getKey());
            if (named instanceof Checked.Rejected<Mechanism> rejected) {
                return Checked.rejected(rejected.rule());
            }
            Mechanism mechanism = Valid.value(named);
            JsonElement way = entry.getValue();
            if (!(way.isJsonPrimitive() && way.getAsJsonPrimitive().isString())) {
                return Checked.rejected("the " + mechanism.asked() + "'s direction is text, not " + way);
            }
            Checked<Boolean> reversed = mechanism.reversedIf(way.getAsString());
            if (reversed instanceof Checked.Rejected<Boolean> rejected) {
                return Checked.rejected(rejected.rule());
            }
            directions = directions.with(mechanism, Valid.value(reversed));
        }
        return Checked.ok(directions);
    }

    static JsonObject directionsJson(Directions directions) {
        JsonObject json = new JsonObject();
        for (Mechanism mechanism : Mechanism.values()) {
            json.addProperty(mechanism.asked(), mechanism.way(directions.reversed(mechanism)));
        }
        return json;
    }

    private static String said(Configuration configuration) {
        List<String> devices = new ArrayList<>();
        for (Configuration.Device device : configuration.devices()) {
            String sentence = device.name() + " " + device.kind();
            if (device instanceof Configuration.Motor motor) {
                sentence += " reading " + motor.encoder();
            }
            devices.add(sentence);
        }
        return String.join(", ", devices);
    }

    private static Checked<JsonObject> fields(
            JsonElement json, String what, List<String> required, List<String> allowed) {
        if (json == null || !json.isJsonObject()) {
            return Checked.rejected(what + " is an object of " + String.join(", ", allowed) + ", not " + json);
        }
        JsonObject object = json.getAsJsonObject();
        Set<String> keys = new HashSet<>();
        for (Map.Entry<String, JsonElement> entry : object.entrySet()) {
            keys.add(entry.getKey());
        }
        for (String key : keys) {
            if (!allowed.contains(key)) {
                return Checked.rejected(what + " names only " + String.join(", ", allowed) + ", not '" + key + "'");
            }
        }
        for (String key : required) {
            if (!keys.contains(key)) {
                return Checked.rejected(
                        what + " names " + String.join(", ", required) + ", and " + key + " is missing");
            }
        }
        return Checked.ok(object);
    }

    private static Checked<String> text(JsonObject object, String key, String what) {
        JsonElement value = object.get(key);
        if (!(value.isJsonPrimitive() && value.getAsJsonPrimitive().isString())) {
            return Checked.rejected(what + "'s " + key + " is text, not " + value);
        }
        return Checked.ok(value.getAsString());
    }

    private static Checked<HubMounting> hub(JsonElement json) {
        String what = "the hub";
        return fields(json, what, HUB_FACES, HUB_FACES)
                .then(object -> text(object, "logo", what)
                        .then(Facing::named)
                        .then(logo ->
                                text(object, "usb", what).then(Facing::named).then(usb -> HubMounting.of(logo, usb))));
    }

    private static JsonObject hubJson(HubMounting hub) {
        JsonObject json = new JsonObject();
        json.addProperty("logo", hub.logo().name());
        json.addProperty("usb", hub.usb().name());
        return json;
    }

    private static Checked<Configuration> configuration(JsonElement json) {
        if (json == null || !json.isJsonArray()) {
            return Checked.rejected("the configuration is a list of devices, not " + json);
        }
        List<Configuration.Device> devices = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray()) {
            Checked<Configuration.Device> device = device(element);
            if (device instanceof Checked.Rejected<Configuration.Device> rejected) {
                return Checked.rejected(rejected.rule());
            }
            devices.add(Valid.value(device));
        }
        return Configuration.of(devices);
    }

    private static Checked<Configuration.Device> device(JsonElement json) {
        String what = "a configured device";
        return fields(json, what, OTHER_FIELDS, MOTOR_FIELDS)
                .then(object -> text(object, "name", what)
                        .then(name -> text(object, "kind", "'" + name + "'").then(kind -> device(object, name, kind))));
    }

    private static Checked<Configuration.Device> device(JsonObject object, String name, String kind) {
        String what = "'" + name + "'";
        Map<String, Function<String, Configuration.Device>> others = Map.of(
                IMU, Configuration.Imu::new,
                CONTINUOUS_SERVO, Configuration.ContinuousServo::new,
                SERVO, Configuration.Servo::new,
                VOLTAGE_SENSOR, Configuration.VoltageSensor::new);
        if (kind.equals(MOTOR)) {
            return fields(object, what, MOTOR_FIELDS, MOTOR_FIELDS)
                    .then(motor -> text(motor, "type", what)
                            .then(MotorType::named)
                            .then(type -> text(motor, "encoder", what)
                                    .then(EncoderReads::named)
                                    .map(encoder -> new Configuration.Motor(name, type, encoder))));
        }
        Function<String, Configuration.Device> other = others.get(kind);
        if (other == null) {
            return Checked.rejected(what + " is a " + String.join(", ", KINDS) + ", not '" + kind + "'");
        }
        return fields(object, what + ", a " + kind + ",", OTHER_FIELDS, OTHER_FIELDS)
                .map(fields -> other.apply(name));
    }

    private static JsonArray configurationJson(Configuration configuration) {
        JsonArray devices = new JsonArray();
        for (Configuration.Device device : configuration.devices()) {
            JsonObject json = new JsonObject();
            json.addProperty("name", device.name());
            json.addProperty("kind", device.kind());
            if (device instanceof Configuration.Motor motor) {
                json.addProperty("type", motor.type().label());
                json.addProperty("encoder", motor.encoder().asked());
            }
            devices.add(json);
        }
        return devices;
    }
}
