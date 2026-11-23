package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class Overrides {
    public enum HubFace {
        LOGO("the hub's logo"),
        USB("the hub's USB ports");

        private final String label;

        HubFace(String label) {
            this.label = label;
        }

        public String label() {
            return label;
        }
    }

    public sealed interface Edit
            permits Edit.OfConstant, Edit.OfHub, Edit.Reversed, Edit.Renamed, Edit.Typed, Edit.Wired {
        String value();

        record OfConstant(Constant constant, double to) implements Edit {
            @Override
            public String value() {
                return constant.label();
            }
        }

        record OfHub(HubFace face, Facing facing) implements Edit {
            @Override
            public String value() {
                return face.label();
            }
        }

        record Reversed(Mechanism mechanism, boolean reversed) implements Edit {
            @Override
            public String value() {
                return mechanism.label() + "'s direction";
            }
        }

        record Renamed(String device, String name) implements Edit {
            @Override
            public String value() {
                return device + "'s name";
            }
        }

        record Typed(String device, MotorType type) implements Edit {
            @Override
            public String value() {
                return device + "'s motor type";
            }
        }

        record Wired(String device, EncoderReads encoder) implements Edit {
            @Override
            public String value() {
                return device + "'s encoder port";
            }
        }
    }

    public sealed interface Kept permits Kept.Known, Kept.Gone, Kept.Unreadable {
        record Known(Edit edit) implements Kept {}

        record Gone(String name) implements Kept {}

        record Unreadable(String name, String why) implements Kept {}
    }

    public enum Fate {
        BACK_TO_DEFAULT("went back to its default"),
        DROPPED_ON_NEXT_SAVE("is dropped on the next save");

        private final String said;

        Fate(String said) {
            this.said = said;
        }
    }

    public record Notice(String value, Fate fate, String why) {
        public String said() {
            return value + " " + fate.said + ": " + why + ".";
        }
    }

    public record Recalled(Overrides overrides, List<Notice> notices) {}

    private sealed interface Admitted permits Admitted.Taken, Admitted.Refused {
        record Taken(Edit edit) implements Admitted {}

        record Refused(Fate fate, String rule) implements Admitted {}
    }

    private static final String EVERY_EDIT = "every edit";
    private static final String NO_SUCH_VALUE_NOW = "the robot description has no such value now";

    private final RobotDescription defaults;
    private final RobotDescription description;

    private Overrides(RobotDescription defaults, RobotDescription description) {
        this.defaults = defaults;
        this.description = description;
    }

    public static Overrides none(RobotDescription defaults) {
        return new Overrides(defaults, defaults);
    }

    public static Checked<Overrides> asked(RobotDescription defaults, List<Kept> kept) {
        List<Edit> edits = new ArrayList<>();
        for (Kept one : kept) {
            if (one instanceof Kept.Gone gone) {
                return Checked.rejected("the robot description has no value named '" + gone.name() + "'");
            }
            if (one instanceof Kept.Unreadable unreadable) {
                return Checked.rejected(unreadable.why());
            }
            if (one instanceof Kept.Known known) {
                Admitted admitted = admitted(defaults, known.edit());
                if (admitted instanceof Admitted.Refused refused) {
                    return Checked.rejected(refused.rule());
                }
                edits.add(known.edit());
            }
        }
        return applied(defaults, edits).map(description -> new Overrides(defaults, description));
    }

    public static Recalled recalled(RobotDescription defaults, List<Kept> kept) {
        List<Edit> edits = new ArrayList<>();
        List<Notice> notices = new ArrayList<>();
        for (Kept one : kept) {
            if (one instanceof Kept.Gone gone) {
                notices.add(new Notice(gone.name(), Fate.DROPPED_ON_NEXT_SAVE, NO_SUCH_VALUE_NOW));
            }
            if (one instanceof Kept.Unreadable unreadable) {
                notices.add(new Notice(unreadable.name(), Fate.BACK_TO_DEFAULT, unreadable.why()));
            }
            if (one instanceof Kept.Known known) {
                Admitted admitted = admitted(defaults, known.edit());
                if (admitted instanceof Admitted.Refused refused) {
                    notices.add(new Notice(known.edit().value(), refused.fate(), refused.rule()));
                } else {
                    edits.add(known.edit());
                }
            }
        }
        return applied(defaults, edits)
                .fold(description -> new Recalled(new Overrides(defaults, description), notices), rule -> {
                    notices.add(new Notice(EVERY_EDIT, Fate.BACK_TO_DEFAULT, rule));
                    return new Recalled(none(defaults), notices);
                });
    }

    private static Admitted admitted(RobotDescription defaults, Edit edit) {
        if (edit instanceof Edit.OfConstant set) {
            return set.constant()
                    .admits(set.to())
                    .fold(value -> new Admitted.Taken(edit), rule -> new Admitted.Refused(Fate.BACK_TO_DEFAULT, rule));
        }
        String device = edit instanceof Edit.Renamed renamed
                ? renamed.device()
                : edit instanceof Edit.Typed typed
                        ? typed.device()
                        : edit instanceof Edit.Wired wired ? wired.device() : "";
        if (edit instanceof Edit.OfHub || edit instanceof Edit.Reversed) {
            return new Admitted.Taken(edit);
        }
        for (Configuration.Device configured : defaults.configuration().devices()) {
            if (configured.name().equals(device)) {
                if (edit instanceof Edit.Renamed || configured instanceof Configuration.Motor) {
                    return new Admitted.Taken(edit);
                }
                return new Admitted.Refused(
                        Fate.BACK_TO_DEFAULT,
                        "'" + device + "' is configured as " + configured.kind() + ", not as a motor, so it has no "
                                + (edit instanceof Edit.Typed ? "motor type" : "encoder port"));
            }
        }
        return new Admitted.Refused(Fate.DROPPED_ON_NEXT_SAVE, "no device is configured as '" + device + "'");
    }

    private static Checked<RobotDescription> applied(RobotDescription defaults, List<Edit> edits) {
        EnumMap<Constant, Double> values = new EnumMap<>(Constant.class);
        for (Constant constant : Constant.values()) {
            values.put(constant, defaults.constants().value(constant));
        }
        EnumMap<HubFace, Facing> faces = new EnumMap<>(HubFace.class);
        faces.put(HubFace.LOGO, defaults.hub().logo());
        faces.put(HubFace.USB, defaults.hub().usb());
        Directions directions = defaults.directions();
        Map<String, Configuration.Device> devices = new LinkedHashMap<>();
        for (Configuration.Device device : defaults.configuration().devices()) {
            devices.put(device.name(), device);
        }
        for (Edit edit : edits) {
            if (edit instanceof Edit.OfConstant set) {
                values.put(set.constant(), set.to());
            }
            if (edit instanceof Edit.OfHub hub) {
                faces.put(hub.face(), hub.facing());
            }
            if (edit instanceof Edit.Reversed reversed) {
                directions = directions.with(reversed.mechanism(), reversed.reversed());
            }
            if (edit instanceof Edit.Renamed renamed) {
                devices.computeIfPresent(renamed.device(), (name, device) -> device.named(renamed.name()));
            }
            if (edit instanceof Edit.Typed typed) {
                devices.computeIfPresent(
                        typed.device(),
                        (name, device) -> device instanceof Configuration.Motor motor
                                ? new Configuration.Motor(motor.name(), typed.type(), motor.encoder())
                                : device);
            }
            if (edit instanceof Edit.Wired wired) {
                devices.computeIfPresent(
                        wired.device(),
                        (name, device) -> device instanceof Configuration.Motor motor
                                ? new Configuration.Motor(motor.name(), motor.type(), wired.encoder())
                                : device);
            }
        }
        List<Configuration.Committed> committed = new ArrayList<>();
        for (Map.Entry<String, Configuration.Device> device : devices.entrySet()) {
            committed.add(new Configuration.Committed(device.getKey(), device.getValue()));
        }
        Facing logo = faces.getOrDefault(HubFace.LOGO, defaults.hub().logo());
        Facing usb = faces.getOrDefault(HubFace.USB, defaults.hub().usb());
        Directions directed = directions;
        return Constants.of(values)
                .then(constants -> HubMounting.of(logo, usb)
                        .then(hub -> Configuration.committedAs(committed)
                                .map(configuration -> RobotDescription.of(configuration, hub, constants, directed))));
    }

    public RobotDescription defaults() {
        return defaults;
    }

    public RobotDescription description() {
        return description;
    }

    public List<Edit> edits() {
        List<Edit> edits = new ArrayList<>();
        for (Constant constant : Constant.values()) {
            double value = description.constants().value(constant);
            if (Double.compare(value, defaults.constants().value(constant)) != 0) {
                edits.add(new Edit.OfConstant(constant, value));
            }
        }
        if (description.hub().logo() != defaults.hub().logo()) {
            edits.add(new Edit.OfHub(HubFace.LOGO, description.hub().logo()));
        }
        if (description.hub().usb() != defaults.hub().usb()) {
            edits.add(new Edit.OfHub(HubFace.USB, description.hub().usb()));
        }
        for (Mechanism mechanism : Mechanism.values()) {
            boolean reversed = description.directions().reversed(mechanism);
            if (reversed != defaults.directions().reversed(mechanism)) {
                edits.add(new Edit.Reversed(mechanism, reversed));
            }
        }
        ArrayDeque<Configuration.Device> byDefault =
                new ArrayDeque<>(defaults.configuration().devices());
        for (Configuration.Device now : description.configuration().devices()) {
            Configuration.Device was = byDefault.pollFirst();
            if (was != null) {
                edits.addAll(deviceEdits(was, now));
            }
        }
        return edits;
    }

    private static List<Edit> deviceEdits(Configuration.Device byDefault, Configuration.Device now) {
        List<Edit> edits = new ArrayList<>();
        String device = byDefault.name();
        if (!now.name().equals(device)) {
            edits.add(new Edit.Renamed(device, now.name()));
        }
        if (byDefault instanceof Configuration.Motor was && now instanceof Configuration.Motor is) {
            if (is.type() != was.type()) {
                edits.add(new Edit.Typed(device, is.type()));
            }
            if (is.encoder() != was.encoder()) {
                edits.add(new Edit.Wired(device, is.encoder()));
            }
        }
        return edits;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Overrides that
                && defaults.equals(that.defaults)
                && description.equals(that.description);
    }

    @Override
    public int hashCode() {
        return 31 * defaults.hashCode() + description.hashCode();
    }
}
