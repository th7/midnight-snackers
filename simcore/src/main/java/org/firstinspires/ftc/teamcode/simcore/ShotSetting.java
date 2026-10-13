package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

public enum ShotSetting {
    X(
            Group.LAUNCHER,
            "Launcher x",
            "in",
            "Where the pollen leaves the wheels, measured from the centre of the field toward the far wall;"
                    + " the audience's side is negative.",
            55,
            -69,
            69),
    Y(
            Group.LAUNCHER,
            "Launcher y",
            "in",
            "Where the pollen leaves the wheels, measured from the centre of the field toward red's wall;"
                    + " blue's side is negative.",
            -12.76,
            -69,
            69),
    HEIGHT(
            Group.LAUNCHER,
            "Exit height",
            "in",
            "How high above the floor the pollen's centre is as it leaves the wheels.",
            14,
            2,
            40),
    YAW(
            Group.LAUNCHER,
            "Yaw",
            "deg",
            "Which way the launcher points, turning left from the far wall: 90 points at red's wall, 180 at the"
                    + " audience and -90 at blue's wall.",
            180,
            -180,
            180),
    PITCH(
            Group.LAUNCHER,
            "Pitch",
            "deg",
            "How steeply the pollen is thrown: 0 is level, 90 is straight up and below 0 is downward.",
            70,
            -30,
            90),
    ROLL(
            Group.LAUNCHER,
            "Roll",
            "deg",
            "Where wheel one sits around the pollen, turning about the way it is thrown: 0 puts it on top, 90 on"
                    + " the left and 180 underneath, and the spin the wheels give turns with it.",
            0,
            -180,
            180),
    WHEEL_ONE_RPM(
            Group.LAUNCHER,
            "Wheel one speed",
            "rpm",
            "How fast wheel one turns. With wheel one on top, a faster wheel one gives topspin, which dips the"
                    + " shot, and a slower one gives backspin, which lifts it.",
            900,
            0,
            6000),
    WHEEL_TWO_RPM(
            Group.LAUNCHER,
            "Wheel two speed",
            "rpm",
            "How fast wheel two, across the pollen from wheel one, turns. The pollen leaves at the two wheels'"
                    + " average surface speed and spins by their difference.",
            1200,
            0,
            6000),
    WHEEL_ONE_DIAMETER(
            Group.LAUNCHER,
            "Wheel one diameter",
            "in",
            "How big wheel one is across: a bigger wheel's surface moves faster at the same rpm.",
            3.78,
            1.5,
            6),
    WHEEL_TWO_DIAMETER(
            Group.LAUNCHER,
            "Wheel two diameter",
            "in",
            "How big wheel two is across: a bigger wheel's surface moves faster at the same rpm.",
            3.78,
            1.5,
            6),
    TICKS_PER_REV(
            Group.LAUNCHER,
            "Encoder ticks per turn",
            "ticks",
            "How many encoder ticks a wheel's motor counts as its wheel turns once. It only changes what the form"
                    + " shows: each wheel's speed in ticks a second, as a motor's setVelocity takes it.",
            28,
            1,
            10000),
    SPEED_EFFICIENCY(
            Group.LAUNCHER,
            "Speed efficiency",
            "",
            "How much of the wheels' average surface speed the pollen leaves with: 1 is all of it, and slip"
                    + " between the wheels and the pollen makes it less.",
            1,
            0.3,
            1),
    SPIN_EFFICIENCY(
            Group.LAUNCHER,
            "Spin efficiency",
            "",
            "How much of the spin the wheels' difference in speed would give the pollen it leaves with: 1 is all"
                    + " of it.",
            1,
            0.3,
            1),
    COMPRESSION(
            Group.LAUNCHER,
            "Compression",
            "in",
            "How much the wheels squeeze the pollen: the gap between them is its diameter less this. A tighter"
                    + " squeeze spins it faster for the same difference in wheel speed.",
            0,
            0,
            0.6),
    WHEELS_SLOW(
            Group.LAUNCHER,
            "Wheels slow down",
            "When on, each wheel loses some of its speed as it throws the pollen, as much as its inertia says,"
                    + " so the pollen leaves slower; when off, the wheels keep their speed.",
            false),
    WHEEL_INERTIA(
            Group.LAUNCHER,
            "Wheel inertia",
            "lb in^2",
            "How hard each wheel, with its flywheel, is to slow down. It counts only when Wheels slow down is on:"
                    + " a heavier wheel loses less speed to the pollen.",
            0.95,
            0.05,
            5),
    ROBOT_VX(
            Group.LAUNCHER,
            "Robot speed x",
            "in/s",
            "How fast the robot is driving toward the far wall as it launches; the pollen leaves with the robot's"
                    + " speed added to its own.",
            0,
            -120,
            120),
    ROBOT_VY(
            Group.LAUNCHER,
            "Robot speed y",
            "in/s",
            "How fast the robot is driving toward red's wall as it launches; the pollen leaves with the robot's"
                    + " speed added to its own.",
            0,
            -120,
            120),
    DIAMETER(
            Group.POLLEN,
            "Pollen diameter",
            "in",
            "How big the launched pollen is across. A bigger ball meets more air, and the wheels spin it less for"
                    + " the same difference in their speeds.",
            2.8,
            2.6,
            3.0),
    MASS(
            Group.POLLEN,
            "Pollen mass",
            "g",
            "How heavy the launched pollen is: 24.9 g is 0.055 lb. The air slows a lighter ball more.",
            24.9,
            20,
            30),
    INERTIA(
            Group.POLLEN,
            "Pollen inertia",
            "of m r^2",
            "How hard the pollen is to set spinning, as a share of its mass times its radius squared: a solid ball"
                    + " is 0.4 and a thin hollow shell 2/3, and the pollen's thick shell makes it 0.634.",
            0.634,
            0.4,
            Ball.THIN_SHELL),
    PIECES(
            Group.POLLEN,
            "Pieces on the field",
            "When on, the pollen and nectar where a match starts are on the field, and the launched pollen can hit"
                    + " them and knock them about; when off, the field is empty.",
            true),
    NECTAR_MASS(
            Group.POLLEN,
            "Nectar mass",
            "g",
            "How heavy each nectar on the field is: 41.3 g is 0.091 lb. A heavier one moves less when the pollen"
                    + " hits it.",
            41.3,
            30,
            55),
    BLUE_HIVE_TIPPED(
            Group.FIELD,
            "Blue hive tipped",
            "When on, blue's hive is tipped the other way, so its other cell is the one that faces up and scores.",
            false),
    RED_HIVE_TIPPED(
            Group.FIELD,
            "Red hive tipped",
            "When on, red's hive is tipped the other way, so its other cell is the one that faces up and scores.",
            false),
    TEMPERATURE(
            Group.AIR,
            "Temperature",
            "C",
            "How warm the air is. Warmer air is thinner, so it holds the pollen back a little less.",
            20,
            -10,
            45),
    ELEVATION(
            Group.AIR,
            "Elevation",
            "m",
            "How high above sea level the field is. The air is thinner higher up, so the pollen flies a little"
                    + " farther.",
            0,
            -100,
            3000),
    HUMIDITY(
            Group.AIR,
            "Humidity",
            "%",
            "How damp the air is, as relative humidity. Damp air is a little thinner than dry air.",
            0,
            0,
            100),
    WIND_X(
            Group.AIR,
            "Wind x",
            "in/s",
            "How fast the air is moving toward the far wall, from a fan or a draught; it carries the pollen along"
                    + " with it.",
            0,
            -200,
            200),
    WIND_Y(
            Group.AIR,
            "Wind y",
            "in/s",
            "How fast the air is moving toward red's wall, from a fan or a draught; it carries the pollen along"
                    + " with it.",
            0,
            -200,
            200),
    DRAG(
            Group.FLIGHT,
            "Drag",
            "",
            "How hard the air holds the pollen back, as its drag coefficient when it is not spinning: 0.45 is a"
                    + " 26-hole pickleball's, measured in flight.",
            0.45,
            0,
            1),
    DRAG_PER_SPIN(
            Group.FLIGHT,
            "Drag per spin",
            "",
            "How much harder the air holds back a spinning pollen: the drag coefficient grows by this for each unit"
                    + " of spin, measured as how fast its surface turns over how fast it flies.",
            0.15,
            0,
            0.5),
    LIFT_SLOPE(
            Group.FLIGHT,
            "Lift slope",
            "",
            "How much lift a little spin makes: the lift coefficient for each unit of spin while the spin is small."
                    + " Lift pushes a backspinning pollen up and a topspinning one down.",
            2,
            0.5,
            5),
    LIFT_LIMIT(
            Group.FLIGHT,
            "Lift limit",
            "",
            "The most lift spin can make, as a lift coefficient: lift grows with spin and levels off toward this.",
            0.4,
            0.1,
            1),
    SPIN_DOWN(
            Group.FLIGHT,
            "Spin-down",
            "",
            "How quickly the air slows the pollen's spin as it flies: the faster it flies, the sooner its spin dies"
                    + " away.",
            0.012,
            0,
            0.05),
    ADDED_MASS(
            Group.FLIGHT,
            "Added mass",
            "",
            "How much of the air around the pollen it drags along, as a share of the air it pushes aside: 0.5 is a"
                    + " sphere's. It makes the ball a little harder to speed up or slow down.",
            0.5,
            0,
            1),
    FLOOR_BOUNCE(Surface.FLOOR, Property.BOUNCE, 0.55),
    FLOOR_FRICTION(Surface.FLOOR, Property.FRICTION, 0.50),
    FLOOR_GRIP(Surface.FLOOR, Property.GRIP, 0.20),
    WALL_BOUNCE(Surface.WALL, Property.BOUNCE, 0.50),
    WALL_FRICTION(Surface.WALL, Property.FRICTION, 0.30),
    WALL_GRIP(Surface.WALL, Property.GRIP, 0.15),
    FRAME_BOUNCE(Surface.FRAME, Property.BOUNCE, 0.62),
    FRAME_FRICTION(Surface.FRAME, Property.FRICTION, 0.35),
    FRAME_GRIP(Surface.FRAME, Property.GRIP, 0.15),
    FLOWER_BOUNCE(Surface.FLOWER, Property.BOUNCE, 0.55),
    FLOWER_FRICTION(Surface.FLOWER, Property.FRICTION, 0.35),
    FLOWER_GRIP(Surface.FLOWER, Property.GRIP, 0.15),
    CELL_BOUNCE(Surface.CELL, Property.BOUNCE, 0.50),
    CELL_FRICTION(Surface.CELL, Property.FRICTION, 0.30),
    CELL_GRIP(Surface.CELL, Property.GRIP, 0.15),
    PIECE_BOUNCE(Surface.PIECE, Property.BOUNCE, 0.55),
    PIECE_FRICTION(Surface.PIECE, Property.FRICTION, 0.30),
    PIECE_GRIP(Surface.PIECE, Property.GRIP, 0.15),
    BOUNCE_LOSS(
            Group.CONTACT,
            "Bounce loss",
            "per 100 in/s",
            "How much less of its speed a pollen keeps from a bounce for each 100 in/s faster it hits than 244 in/s,"
                    + " and how much more for each 100 in/s slower.",
            0.038,
            0,
            0.2),
    BOUNCE_THRESHOLD(
            Group.CONTACT,
            "Bounce threshold",
            "in/s",
            "The slowest a pollen can hit something and still bounce off it; a slower hit settles against it.",
            4,
            0.5,
            25),
    IMPACT_TWIST(
            Group.CONTACT,
            "Impact twist",
            "in",
            "How much a bounce slows the pollen's spin about the line to what it hits, like a top's: the lever the"
                    + " rub at the contact works with.",
            0.1,
            0,
            0.3),
    SPIN_TWIST(
            Group.CONTACT,
            "Spin twist",
            "in",
            "How quickly a pollen resting or rolling on something stops spinning on it like a top: the lever the"
                    + " rub at the contact works with.",
            0.06,
            0,
            0.2),
    ROLLING_RESISTANCE(
            Group.CONTACT,
            "Rolling resistance",
            "",
            "How quickly a rolling pollen slows, as a share of gravity: 0.04 takes about 15 in/s off its speed"
                    + " every second.",
            0.04,
            0,
            0.2),
    STEP(
            Group.SIMULATION,
            "Step",
            "ms",
            "How much time the simulation moves on at once. A shorter step is more exact and takes more work, and a"
                    + " shot that takes too much work is stopped.",
            0.1,
            0.01,
            2),
    LONGEST(
            Group.SIMULATION,
            "Longest shot",
            "s",
            "The longest a shot is followed; a pollen still moving then has run out of time.",
            8,
            0.5,
            20);

    public enum Group {
        LAUNCHER("Launcher", "Where the launcher is, which way it points, and how its two wheels throw.", true),
        POLLEN("Pollen and pieces", "The pollen that is launched, and the pieces on the field it may hit.", true),
        FIELD("Field", "How the field is set up: whether either hive has been tipped.", true),
        AIR("Air", "The air the pollen flies through: how thick it is, and any wind.", false),
        FLIGHT(
                "Flight model",
                "How the air acts on a flying pollen: drag slows it, spin lifts it or dips it, and the air slows the"
                        + " spin.",
                false),
        BOUNCE(
                "Bounce",
                "How a pollen bounces off each surface: how much speed it keeps, how hard the surface rubs on it, and"
                        + " how much of its spin springs back.",
                false),
        CONTACT(
                "Contact",
                "What every contact shares: how bounce falls off with speed, when a hit is too slow to bounce, and how"
                        + " a pollen that is touching something stops spinning and rolling.",
                false),
        SIMULATION("Simulation", "How finely the shot is worked out, and for how long.", false);

        private final String label;
        private final String says;
        private final boolean openByDefault;

        Group(String label, String says, boolean openByDefault) {
            this.label = label;
            this.says = says;
            this.openByDefault = openByDefault;
        }

        public String asked() {
            return name().toLowerCase(Locale.ROOT);
        }

        public String label() {
            return label;
        }

        public String says() {
            return says;
        }

        public boolean openByDefault() {
            return openByDefault;
        }
    }

    public enum Property {
        BOUNCE("Bounce", "How much of its speed into a surface a pollen keeps as it bounces back.", 1),
        FRICTION("Friction", "How hard a surface rubs on a pollen that slides along it as they touch.", 1.5),
        GRIP("Grip", "How much of a pollen's slide and spin springs back the other way when a surface grips it.", 1);

        private final String label;
        private final String says;
        private final double most;

        Property(String label, String says, double most) {
            this.label = label;
            this.says = says;
            this.most = most;
        }

        public String asked() {
            return name().toLowerCase(Locale.ROOT);
        }

        public String label() {
            return label;
        }

        public String says() {
            return says;
        }

        double most() {
            return most;
        }
    }

    public sealed interface Kind permits Amount, Switch, OnSurface {}

    public record Amount() implements Kind {}

    public record Switch() implements Kind {}

    public record OnSurface(Surface surface, Property property) implements Kind {}

    private final Group group;
    private final String label;
    private final String unit;
    private final String says;
    private final double byDefault;
    private final double least;
    private final double most;
    private final boolean isSwitch;
    private final Optional<Surface> surface;
    private final Optional<Property> property;

    ShotSetting(Group group, String label, String unit, String says, double byDefault, double least, double most) {
        this.group = group;
        this.label = label;
        this.unit = unit;
        this.says = says;
        this.byDefault = byDefault;
        this.least = least;
        this.most = most;
        this.isSwitch = false;
        this.surface = Optional.empty();
        this.property = Optional.empty();
    }

    ShotSetting(Group group, String label, String says, boolean onByDefault) {
        this.group = group;
        this.label = label;
        this.unit = "";
        this.says = says;
        this.byDefault = onByDefault ? 1 : 0;
        this.least = 0;
        this.most = 1;
        this.isSwitch = true;
        this.surface = Optional.empty();
        this.property = Optional.empty();
    }

    ShotSetting(Surface surface, Property property, double byDefault) {
        this.group = Group.BOUNCE;
        this.label = labelOf(surface) + " " + property.label().toLowerCase(Locale.ROOT);
        this.unit = "";
        this.says = saysOf(surface, property);
        this.byDefault = byDefault;
        this.least = 0;
        this.most = property.most();
        this.isSwitch = false;
        this.surface = Optional.of(surface);
        this.property = Optional.of(property);
    }

    private static String labelOf(Surface surface) {
        return switch (surface) {
            case FLOOR -> "Floor";
            case WALL -> "Wall";
            case FRAME -> "Frame";
            case FLOWER -> "Flower";
            case CELL -> "Cell";
            case PIECE -> "Ball on ball";
        };
    }

    private static String struck(Surface surface) {
        return switch (surface) {
            case FLOOR -> "the floor";
            case WALL -> "a wall";
            case FRAME -> "the metal frame";
            case FLOWER -> "a flower";
            case CELL -> "a cell";
            case PIECE -> "another ball";
        };
    }

    private static String saysOf(Surface surface, Property property) {
        String struck = struck(surface);
        return switch (property) {
            case BOUNCE ->
                "How much of its speed into " + struck + " a pollen keeps as it bounces back, when it hits"
                        + " at 244 in/s: 1 keeps it all and 0 none, and a harder hit keeps a little less.";
            case FRICTION ->
                "How hard " + struck + " rubs on a pollen sliding along it as they touch, as a friction"
                        + " coefficient: the harder the rub, the more of the slide turns into spin.";
            case GRIP ->
                "How much of a pollen's slide and spin along " + struck + " springs back the other way when"
                        + " the bounce grips: 0 leaves it rolling and 1 throws it all back.";
        };
    }

    public static Checked<ShotSetting> named(String asked) {
        for (ShotSetting setting : values()) {
            if (setting.asked().equals(asked)) {
                return Checked.ok(setting);
            }
        }
        return Checked.rejected("a launch setting is " + everyName() + ", not '" + asked + "'");
    }

    private static String everyName() {
        List<String> names = new ArrayList<>();
        for (ShotSetting setting : values()) {
            names.add(setting.asked());
        }
        return String.join(", ", names);
    }

    public Checked<Double> admits(double value) {
        if (isSwitch && !(value == least || value == most)) {
            return Checked.rejected(label + " is " + least + " for off or " + most + " for on, not " + value);
        }
        if (!(value >= least && value <= most)) {
            return Checked.rejected(label + " is from " + least + " to " + most + inUnit() + ", not " + value);
        }
        return Checked.ok(withoutNegativeZero(value));
    }

    private String inUnit() {
        return unit.isEmpty() ? "" : " " + unit;
    }

    private static double withoutNegativeZero(double value) {
        return value + 0.0;
    }

    public String asked() {
        return name().toLowerCase(Locale.ROOT);
    }

    public Group group() {
        return group;
    }

    public String label() {
        return label;
    }

    public String unit() {
        return unit;
    }

    public String says() {
        return says;
    }

    public double byDefault() {
        return byDefault;
    }

    public double least() {
        return least;
    }

    public double most() {
        return most;
    }

    public Kind kind() {
        if (isSwitch) {
            return new Switch();
        }
        Optional<Kind> onSurface = surface.flatMap(on -> property.map(what -> new OnSurface(on, what)));
        return onSurface.orElse(new Amount());
    }
}
