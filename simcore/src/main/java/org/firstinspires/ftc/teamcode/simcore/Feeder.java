package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class Feeder implements Surroundings.Part {
    static final String CAD_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: the Onshape part studio Feeder"
            + " (b57b76498365b6a963155c10, e/0296dc8087bbd2d78083cd41) read on 16 November, in mm in the feeder"
            + " frame: z up the feeder's axis from the floor slab's underside, -y toward the robot's back, +x toward"
            + " its right";

    static final String FLOOR = "Feeder floor";
    static final String WALL = "Feeder wall";
    static final String TOP_PLATE = "Feeder top plate";
    static final String HUB = "Spinner hub";
    static final String FIRST_ARM = "Spinner's first arm";
    static final String SECOND_ARM = "Spinner's second arm";
    static final String RAMP_BAR = "Ramp bar";
    static final String ROBOT_CENTRE_SOURCE = "Launcher's frame is centred 204 mm behind Reginald's front, x forward, y"
            + " left, z up: its axle, at Reginald's Y 217, is AXLE_BEHIND_CENTRE_MM behind that";
    static final double ROBOT_CENTRE_BACK_MM = 204;

    private final Faces faces;
    private final Placing placing;
    private final Motion motion;
    private final GridTime time;
    private final double clockwise;
    private final double raised;
    private final Vec3 spinnerSpin;
    private final Vec3 rampSpin;
    private final Vec3 pivot;

    private Feeder(Faces faces, Placing placing, Motion motion, GridTime time) {
        this.faces = faces;
        this.placing = placing;
        this.motion = motion;
        this.time = time;
        this.clockwise = motion.clockwiseAt(time);
        this.raised = motion.raisedAt(time);
        this.spinnerSpin = placing.direction(new Vec3(0, 0, -motion.clockwiseRate));
        this.rampSpin = placing.direction(new Vec3(0, motion.raisedRate, 0));
        this.pivot = placing.world(new Vec3(faces.pivotX, 0, faces.pivotZ));
    }

    static Checked<Feeder> placed(Shape shape, Launcher.Frame frame, Motion motion) {
        return Faces.of(shape).map(faces -> new Feeder(faces, Placing.of(shape, frame), motion, motion.began));
    }

    Feeder moved(Motion next) {
        return new Feeder(faces, placing, next, next.began);
    }

    Motion motion() {
        return motion;
    }

    record Loads(Torque spinner, Torque ramp) {
        static Loads none() {
            return new Loads(new Torque(0), new Torque(0));
        }
    }

    static Set<String> tallied() {
        Set<String> faces = new HashSet<>();
        faces.add(HUB);
        faces.add(FIRST_ARM);
        faces.add(SECOND_ARM);
        faces.add(RAMP_BAR);
        return Collections.unmodifiableSet(faces);
    }

    Loads loads(Map<String, Bodies.Tally> tallies, Seconds over) {
        Bodies.Tally spinner = new Bodies.Tally(Vec3.zero(), Vec3.zero());
        Bodies.Tally ramp = new Bodies.Tally(Vec3.zero(), Vec3.zero());
        for (Map.Entry<String, Bodies.Tally> tally : tallies.entrySet()) {
            if (tally.getKey().equals(RAMP_BAR)) {
                ramp = together(ramp, tally.getValue());
            } else if (ofTheSpinner(tally.getKey())) {
                spinner = together(spinner, tally.getValue());
            }
        }
        return new Loads(
                torque(spinner, placing.world(Vec3.zero()), placing.direction(new Vec3(0, 0, -1)), over),
                torque(ramp, pivot, placing.direction(new Vec3(0, 1, 0)), over));
    }

    boolean touchesTheSpinner(Vec3 centre, Length radius) {
        for (Surroundings.Touch touch : near(centre, radius, Shot.TOUCHING_IN)) {
            if (ofTheSpinner(touch.id())) {
                return true;
            }
        }
        return false;
    }

    private static boolean ofTheSpinner(String id) {
        return id.equals(HUB) || id.equals(FIRST_ARM) || id.equals(SECOND_ARM);
    }

    private static Bodies.Tally together(Bodies.Tally one, Bodies.Tally other) {
        return new Bodies.Tally(
                one.impulse().plus(other.impulse()), one.moment().plus(other.moment()));
    }

    private static Torque torque(Bodies.Tally given, Vec3 about, Vec3 axis, Seconds over) {
        double inchSquaredKilogramsPerSecond =
                given.moment().minus(about.cross(given.impulse())).dot(axis);
        return Torque.ofNewtonMetres(
                        inchSquaredKilogramsPerSecond * Length.METRES_PER_INCH * Length.METRES_PER_INCH / over.value())
                .orElse(new Torque(0));
    }

    Vec3 againstTheWall(Length pollen, double degreesFromTheBack) {
        double turn = StrictMath.toRadians(degreesFromTheBack);
        double centre = faces.wallRadius - pollen.inches();
        return placing.world(new Vec3(
                -centre * StrictMath.sin(turn), -centre * StrictMath.cos(turn), faces.floorTop + pollen.inches()));
    }

    static final class Motion {
        private final GridTime began;
        private final double clockwise;
        private final double clockwiseRate;
        private final double raised;
        private final double raisedRate;

        private Motion(GridTime began, double clockwise, double clockwiseRate, double raised, double raisedRate) {
            this.began = began;
            this.clockwise = clockwise;
            this.clockwiseRate = clockwiseRate;
            this.raised = raised;
            this.raisedRate = raisedRate;
        }

        static Motion still(GridTime at, Spinner.Turning spinner, RampAngle ramp) {
            return new Motion(at, spinner.radians(), 0, StrictMath.toRadians(ramp.degrees()), 0);
        }

        static Checked<Motion> of(
                GridTime began,
                Seconds over,
                Spinner.Turning spinnerFrom,
                Spinner.Turning spinnerTo,
                RampAngle rampFrom,
                RampAngle rampTo) {
            double seconds = over.value();
            if (!(seconds > 0)) {
                return Checked.rejected("a feeder's spinner and ramp move over more than 0 s, not " + seconds);
            }
            double from = StrictMath.toRadians(rampFrom.degrees());
            return Checked.ok(new Motion(
                    began,
                    spinnerFrom.radians(),
                    (spinnerTo.radians() - spinnerFrom.radians()) / seconds,
                    from,
                    (StrictMath.toRadians(rampTo.degrees()) - from) / seconds));
        }

        private double since(GridTime time) {
            return time.seconds() - began.seconds();
        }

        double clockwiseAt(GridTime time) {
            return clockwise + clockwiseRate * since(time);
        }

        double raisedAt(GridTime time) {
            return raised + raisedRate * since(time);
        }

        @Override
        public String toString() {
            return "from " + began + " the spinner at " + clockwise + " rad clockwise turning " + clockwiseRate
                    + " rad/s and the ramp at " + raised + " rad rising " + raisedRate + " rad/s";
        }
    }

    private static final class Placing {
        private final Vec3 origin;
        private final Vec3 x;
        private final Vec3 y;
        private final Vec3 z;

        private Placing(Vec3 origin, Vec3 x, Vec3 y, Vec3 z) {
            this.origin = origin;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        static Placing of(Shape shape, Launcher.Frame frame) {
            double cos = StrictMath.cos(frame.yaw());
            double sin = StrictMath.sin(frame.yaw());
            Vec3 middle = shape.mount.inReginald(Vec3.zero());
            Vec3 local = new Vec3(ROBOT_CENTRE_BACK_MM - middle.y(), middle.x(), middle.z())
                    .times(1 / Launcher.MILLIMETRES_PER_INCH);
            Vec3 origin = new Vec3(frame.x(), frame.y(), 0).plus(turned(local, cos, sin));
            return new Placing(
                    origin,
                    axis(shape, new Vec3(1, 0, 0), middle, cos, sin),
                    axis(shape, new Vec3(0, 1, 0), middle, cos, sin),
                    axis(shape, new Vec3(0, 0, 1), middle, cos, sin));
        }

        private static Vec3 axis(Shape shape, Vec3 feeder, Vec3 middle, double cos, double sin) {
            Vec3 along = shape.mount.inReginald(feeder).minus(middle);
            return turned(new Vec3(-along.y(), along.x(), along.z()), cos, sin);
        }

        private static Vec3 turned(Vec3 local, double cos, double sin) {
            return new Vec3(cos * local.x() - sin * local.y(), sin * local.x() + cos * local.y(), local.z());
        }

        Vec3 feeder(Vec3 world) {
            Vec3 off = world.minus(origin);
            return new Vec3(off.dot(x), off.dot(y), off.dot(z));
        }

        Vec3 direction(Vec3 feeder) {
            return x.times(feeder.x()).plus(y.times(feeder.y())).plus(z.times(feeder.z()));
        }

        Vec3 world(Vec3 feeder) {
            return origin.plus(direction(feeder));
        }
    }

    private enum Mover {
        STILL,
        SPINNER,
        RAMP
    }

    private record Found(String id, Vec3 normal, double gap, Mover mover) {}

    private record Opening(
            double halfWidth, double sill, double side, double cornerY, String sillId, String left, String right) {
        boolean holds(Vec3 onTheWall) {
            return StrictMath.abs(onTheWall.x()) < halfWidth && onTheWall.y() * side > 0 && onTheWall.z() > sill;
        }
    }

    private static final class Faces {
        private final double floorTop;
        private final double wallRadius;
        private final double plateUnderside;
        private final List<Opening> openings;
        private final double backFlatY;
        private final double frontFlatY;
        private final double notchHalfWidth;
        private final double notchInnerEndY;
        private final double slotFromX;
        private final double slotToX;
        private final double slotFromY;
        private final double slotToY;
        private final double hubRadius;
        private final double armBottom;
        private final double armTop;
        private final double armFarthest;
        private final Collider firstArm;
        private final Collider secondArm;
        private final double pivotX;
        private final double pivotZ;
        private final double rounding;
        private final double barFarthest;
        private final Collider bar;

        private Faces(Shape shape, Collider firstArm, Collider secondArm, Collider bar) {
            Room room = shape.room;
            this.floorTop = inches(room.floorTop);
            this.wallRadius = inches(room.wallRadius);
            this.plateUnderside = inches(room.plateUnderside);
            this.openings = List.of(opening(shape.front, 1, "front"), opening(shape.back, -1, "back"));
            this.backFlatY = inches(shape.plate.backFlatY);
            this.frontFlatY = inches(shape.plate.frontFlatY);
            this.notchHalfWidth = inches(shape.plate.notchHalfWidth);
            this.notchInnerEndY = inches(shape.plate.notchInnerEndY);
            this.slotFromX = inches(shape.slot.fromX);
            this.slotToX = inches(shape.slot.toX);
            this.slotFromY = inches(shape.slot.fromY);
            this.slotToY = inches(shape.slot.toY);
            Arms arms = shape.arms;
            this.hubRadius = inches(arms.hubRadius);
            this.armBottom = inches(arms.armBottom);
            this.armTop = inches(arms.armTop);
            this.armFarthest =
                    inches(StrictMath.max(arms.hubRadius, StrictMath.hypot(arms.armReach, arms.armThickness / 2)));
            this.firstArm = firstArm;
            this.secondArm = secondArm;
            Bar drawn = shape.bar;
            this.pivotX = inches(drawn.pivotX);
            this.pivotZ = inches(drawn.pivotZ);
            this.rounding = inches(drawn.rounding);
            double farthest = 0;
            for (double x : new double[] {drawn.freeEndX, drawn.padX}) {
                for (double z : new double[] {drawn.topAsDrawn, drawn.topAsDrawn - drawn.height}) {
                    farthest = StrictMath.max(farthest, StrictMath.hypot(x - drawn.pivotX, z - drawn.pivotZ));
                }
            }
            this.barFarthest = inches(farthest);
            this.bar = bar;
        }

        static Checked<Faces> of(Shape shape) {
            Arms arms = shape.arms;
            double half = inches(arms.armThickness / 2);
            double reach = inches(arms.armReach);
            double bottom = inches(arms.armBottom);
            double top = inches(arms.armTop);
            Bar drawn = shape.bar;
            double round = drawn.rounding;
            return Collider.box(FIRST_ARM, Surface.FRAME, new Vec3(-half, 0, bottom), new Vec3(half, reach, top))
                    .then(first -> Collider.box(
                                    SECOND_ARM, Surface.FRAME, new Vec3(-half, -reach, bottom), new Vec3(half, 0, top))
                            .then(second -> Collider.box(
                                            RAMP_BAR,
                                            Surface.FRAME,
                                            new Vec3(
                                                            drawn.freeEndX + round,
                                                            drawn.middleY - drawn.width / 2 + round,
                                                            drawn.topAsDrawn - drawn.height + round)
                                                    .times(1 / Launcher.MILLIMETRES_PER_INCH),
                                            new Vec3(
                                                            drawn.padX - round,
                                                            drawn.middleY + drawn.width / 2 - round,
                                                            drawn.topAsDrawn - round)
                                                    .times(1 / Launcher.MILLIMETRES_PER_INCH))
                                    .map(bar -> new Faces(shape, first, second, bar))));
        }

        private Opening opening(Window window, double side, String where) {
            double halfWidth = inches(window.halfWidth);
            return new Opening(
                    halfWidth,
                    inches(window.sill),
                    side,
                    StrictMath.sqrt(wallRadius * wallRadius - halfWidth * halfWidth),
                    "Feeder " + where + " sill",
                    "Feeder " + where + " window's left side",
                    "Feeder " + where + " window's right side");
        }

        List<String> ids() {
            List<String> ids = new ArrayList<>();
            ids.add(FLOOR);
            ids.add(WALL);
            for (Opening opening : openings) {
                ids.add(opening.sillId());
                ids.add(opening.left());
                ids.add(opening.right());
            }
            ids.add(TOP_PLATE);
            ids.add(HUB);
            ids.add(FIRST_ARM);
            ids.add(SECOND_ARM);
            ids.add(RAMP_BAR);
            return ids;
        }

        boolean reaches(Vec3 centre, double by) {
            double lowest = StrictMath.min(floorTop, pivotZ - barFarthest);
            return StrictMath.hypot(centre.x(), centre.y()) <= wallRadius + by
                    && centre.z() >= lowest - by
                    && centre.z() <= plateUnderside + by;
        }

        double sweep(double clockwiseRate, double raisedRate) {
            return StrictMath.abs(clockwiseRate) * armFarthest + StrictMath.abs(raisedRate) * barFarthest;
        }

        List<Found> near(Vec3 centre, double radius, double within, double clockwise, double raised) {
            List<Found> found = new ArrayList<>();
            floor(found, centre, radius, within);
            wall(found, centre, radius, within);
            plate(found, centre, radius, within);
            spinner(found, centre, radius, within, clockwise);
            ramp(found, centre, radius, within, raised);
            return found;
        }

        private static void nearest(
                List<Found> found,
                String id,
                Vec3 centre,
                Vec3 nearest,
                Vec3 otherwise,
                double radius,
                double within,
                Mover mover) {
            Vec3 off = centre.minus(nearest);
            double distance = StrictMath.sqrt(off.dot(off));
            double gap = distance - radius;
            if (gap <= within) {
                found.add(new Found(id, distance > 0 ? off.times(1 / distance) : otherwise, gap, mover));
            }
        }

        private void floor(List<Found> found, Vec3 centre, double radius, double within) {
            double x = centre.x(), y = centre.y();
            double out = StrictMath.hypot(x, y);
            if (out > wallRadius) {
                x = x * wallRadius / out;
                y = y * wallRadius / out;
            } else if (x > slotFromX && x < slotToX && y > slotFromY && y < slotToY) {
                double toFromX = x - slotFromX, toToX = slotToX - x, toFromY = y - slotFromY, toToY = slotToY - y;
                double least = StrictMath.min(StrictMath.min(toFromX, toToX), StrictMath.min(toFromY, toToY));
                if (least == toFromX) {
                    x = slotFromX;
                } else if (least == toToX) {
                    x = slotToX;
                } else if (least == toFromY) {
                    y = slotFromY;
                } else {
                    y = slotToY;
                }
            }
            nearest(found, FLOOR, centre, new Vec3(x, y, floorTop), new Vec3(0, 0, 1), radius, within, Mover.STILL);
        }

        private void wall(List<Found> found, Vec3 centre, double radius, double within) {
            double out = StrictMath.hypot(centre.x(), centre.y());
            double alongX = out > 0 ? centre.x() / out : 1, alongY = out > 0 ? centre.y() / out : 0;
            Vec3 onTheWall =
                    new Vec3(alongX * wallRadius, alongY * wallRadius, clamp(centre.z(), floorTop, plateUnderside));
            Vec3 inward = new Vec3(-alongX, -alongY, 0);
            for (Opening opening : openings) {
                if (opening.holds(onTheWall)) {
                    nearest(
                            found,
                            opening.sillId(),
                            centre,
                            new Vec3(onTheWall.x(), onTheWall.y(), opening.sill()),
                            inward,
                            radius,
                            within,
                            Mover.STILL);
                    double z = clamp(centre.z(), opening.sill(), plateUnderside);
                    double y = opening.side() * opening.cornerY();
                    nearest(
                            found,
                            opening.left(),
                            centre,
                            new Vec3(-opening.halfWidth(), y, z),
                            inward,
                            radius,
                            within,
                            Mover.STILL);
                    nearest(
                            found,
                            opening.right(),
                            centre,
                            new Vec3(opening.halfWidth(), y, z),
                            inward,
                            radius,
                            within,
                            Mover.STILL);
                    return;
                }
            }
            nearest(found, WALL, centre, onTheWall, inward, radius, within, Mover.STILL);
        }

        private void plate(List<Found> found, Vec3 centre, double radius, double within) {
            double x = centre.x(), y = clamp(centre.y(), backFlatY, frontFlatY);
            if (StrictMath.abs(x) < notchHalfWidth && y < notchInnerEndY) {
                double endX = clamp(centre.x(), -notchHalfWidth, notchHalfWidth);
                double sideX = centre.x() < 0 ? -notchHalfWidth : notchHalfWidth;
                double sideY = clamp(centre.y(), backFlatY, notchInnerEndY);
                double toEnd = StrictMath.hypot(centre.x() - endX, centre.y() - notchInnerEndY);
                double toSide = StrictMath.hypot(centre.x() - sideX, centre.y() - sideY);
                x = toEnd <= toSide ? endX : sideX;
                y = toEnd <= toSide ? notchInnerEndY : sideY;
            }
            nearest(
                    found,
                    TOP_PLATE,
                    centre,
                    new Vec3(x, y, plateUnderside),
                    new Vec3(0, 0, -1),
                    radius,
                    within,
                    Mover.STILL);
        }

        private void spinner(List<Found> found, Vec3 centre, double radius, double within, double clockwise) {
            double out = StrictMath.hypot(centre.x(), centre.y());
            double across = out - hubRadius;
            double above = StrictMath.max(armBottom - centre.z(), centre.z() - armTop);
            if (across <= 0 && above <= 0) {
                double gap = StrictMath.max(across, above) - radius;
                if (gap <= within) {
                    Vec3 normal = across > above
                            ? (out > 0 ? new Vec3(centre.x() / out, centre.y() / out, 0) : new Vec3(1, 0, 0))
                            : new Vec3(0, 0, centre.z() > armTop ? 1 : -1);
                    found.add(new Found(HUB, normal, gap, Mover.SPINNER));
                }
            } else {
                double reach = StrictMath.min(out, hubRadius);
                Vec3 nearest = new Vec3(
                        out > 0 ? centre.x() * reach / out : 0,
                        out > 0 ? centre.y() * reach / out : 0,
                        clamp(centre.z(), armBottom, armTop));
                nearest(found, HUB, centre, nearest, new Vec3(0, 0, 1), radius, within, Mover.SPINNER);
            }
            double cos = StrictMath.cos(clockwise), sin = StrictMath.sin(clockwise);
            Vec3 turned =
                    new Vec3(centre.x() * cos - centre.y() * sin, centre.x() * sin + centre.y() * cos, centre.z());
            for (Collider arm : List.of(firstArm, secondArm)) {
                if (arm.reaches(turned, radius + within)) {
                    Colliders.Near near = arm.near(turned, radius);
                    if (near.gap() <= within) {
                        Vec3 n = near.normal();
                        found.add(new Found(
                                arm.name(),
                                new Vec3(n.x() * cos + n.y() * sin, -n.x() * sin + n.y() * cos, n.z()),
                                near.gap(),
                                Mover.SPINNER));
                    }
                }
            }
        }

        private void ramp(List<Found> found, Vec3 centre, double radius, double within, double raised) {
            double cos = StrictMath.cos(raised), sin = StrictMath.sin(raised);
            double out = centre.x() - pivotX, up = centre.z() - pivotZ;
            Vec3 drawn = new Vec3(pivotX + out * cos - up * sin, centre.y(), pivotZ + out * sin + up * cos);
            double reach = radius + rounding;
            if (bar.reaches(drawn, reach + within)) {
                Colliders.Near near = bar.near(drawn, reach);
                if (near.gap() <= within) {
                    Vec3 n = near.normal();
                    found.add(new Found(
                            RAMP_BAR,
                            new Vec3(n.x() * cos + n.z() * sin, n.y(), n.z() * cos - n.x() * sin),
                            near.gap(),
                            Mover.RAMP));
                }
            }
        }

        private static double clamp(double value, double least, double most) {
            return StrictMath.max(least, StrictMath.min(most, value));
        }

        private static double inches(double millimetres) {
            return millimetres / Launcher.MILLIMETRES_PER_INCH;
        }
    }

    @Override
    public List<String> ids() {
        return faces.ids();
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        return faces.reaches(placing.feeder(centre), by);
    }

    @Override
    public double sweep() {
        return faces.sweep(motion.clockwiseRate, motion.raisedRate);
    }

    @Override
    public boolean moving() {
        return motion.clockwiseRate != 0 || motion.raisedRate != 0;
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        List<Surroundings.Touch> touches = new ArrayList<>();
        double r = radius.inches();
        Vec3 inFeeder = placing.feeder(centre);
        if (!faces.reaches(inFeeder, r + within)) {
            return touches;
        }
        for (Found found : faces.near(inFeeder, r, within, clockwise, raised)) {
            Vec3 normal = placing.direction(found.normal());
            Vec3 contact = centre.along(normal, -(r + found.gap()));
            Vec3 spin =
                    found.mover() == Mover.SPINNER ? spinnerSpin : found.mover() == Mover.RAMP ? rampSpin : Vec3.zero();
            Vec3 about = found.mover() == Mover.RAMP ? pivot : placing.origin;
            Vec3 velocity = spin.cross(contact.minus(about));
            Surroundings.Touch.of(found.id(), Surface.FRAME, normal, found.gap(), velocity, spin)
                    .fold(touch -> touches.add(touch), rule -> false);
        }
        return touches;
    }

    @Override
    public boolean squeezes() {
        return false;
    }

    @Override
    public List<Surroundings.Overlap> squeezing(Vec3 centre, Length radius) {
        return List.of();
    }

    @Override
    public Feeder at(GridTime then) {
        return new Feeder(faces, placing, motion, then);
    }

    @Override
    public String toString() {
        return "the feeder at " + time + ", " + motion;
    }

    private static boolean finite(double... measures) {
        for (double measure : measures) {
            if (!Double.isFinite(measure)) {
                return false;
            }
        }
        return true;
    }

    public static final class Torque {
        private final double newtonMetres;

        private Torque(double newtonMetres) {
            this.newtonMetres = newtonMetres;
        }

        public static Checked<Torque> ofNewtonMetres(double newtonMetres) {
            if (!Double.isFinite(newtonMetres)) {
                return Checked.rejected("a torque is a finite number of N m, not " + newtonMetres);
            }
            return Checked.ok(new Torque(newtonMetres));
        }

        public double newtonMetres() {
            return newtonMetres;
        }

        Torque negated() {
            return new Torque(-newtonMetres);
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Torque that && Double.compare(newtonMetres, that.newtonMetres) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(newtonMetres);
        }

        @Override
        public String toString() {
            return newtonMetres + " N m";
        }
    }

    public static final class Spinner {
        static final String SERVO_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the REV Smart Robot Servo"
                + " (REV-41-1097) in continuous mode, 0.14 s per 60 degrees and 13.5 kg cm at 6 V by the vendor,"
                + " scaled to the Hub's 5 V: 1.0 rev/s unloaded and 1.1 N m at stall";
        static final double FULL_SPEED_REV_PER_S = 1.0;
        static final double STALL_NEWTON_METRES = 1.1;
        static final String RANGE_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the spinner's speed at full"
                + " power is set from 0.2 to 3 rev/s, and its stall torque from 0.1 to 5 N m";
        static final double LEAST_FULL_SPEED_REV_PER_S = 0.2;
        static final double MOST_FULL_SPEED_REV_PER_S = 3;
        static final double LEAST_STALL_NEWTON_METRES = 0.1;
        static final double MOST_STALL_NEWTON_METRES = 5;

        private final double fullSpeed;
        private final double stall;

        private Spinner(double fullSpeed, double stall) {
            this.fullSpeed = fullSpeed;
            this.stall = stall;
        }

        public static Checked<Spinner> of(double fullSpeedRevPerS, double stallNewtonMetres) {
            if (!(fullSpeedRevPerS >= LEAST_FULL_SPEED_REV_PER_S && fullSpeedRevPerS <= MOST_FULL_SPEED_REV_PER_S)) {
                return Checked.rejected(
                        "a spinner's speed at full power, unloaded, is from " + LEAST_FULL_SPEED_REV_PER_S + " to "
                                + MOST_FULL_SPEED_REV_PER_S + " rev/s, not " + fullSpeedRevPerS);
            }
            if (!(stallNewtonMetres >= LEAST_STALL_NEWTON_METRES && stallNewtonMetres <= MOST_STALL_NEWTON_METRES)) {
                return Checked.rejected("a spinner's stall torque is from " + LEAST_STALL_NEWTON_METRES + " to "
                        + MOST_STALL_NEWTON_METRES + " N m, not " + stallNewtonMetres);
            }
            return Checked.ok(new Spinner(fullSpeedRevPerS, stallNewtonMetres));
        }

        public static Spinner revSmartRobotServo() {
            return new Spinner(FULL_SPEED_REV_PER_S, STALL_NEWTON_METRES);
        }

        public Turning next(Turning from, Power power, Torque load, Seconds step) {
            double speed = fullSpeed * power.value() * StrictMath.max(0, 1 - load.newtonMetres() / stall);
            return new Turning(from.radians + speed * Flywheel.WHOLE_TURN * step.value(), speed);
        }

        @Override
        public String toString() {
            return "a spinner servo turning " + fullSpeed + " rev/s unloaded, stalling at " + stall + " N m";
        }

        public static final class Turning {
            private final double radians;
            private final double revolutionsPerSecond;

            private Turning(double radians, double revolutionsPerSecond) {
                this.radians = radians;
                this.revolutionsPerSecond = revolutionsPerSecond;
            }

            public static Turning still() {
                return new Turning(0, 0);
            }

            public double radians() {
                return radians;
            }

            public double revolutionsPerSecond() {
                return revolutionsPerSecond;
            }
        }
    }

    public static final class Ramp {
        static final String SERVO_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the REV Smart Robot Servo"
                + " (REV-41-1097) in angular mode, 0.14 s per 60 degrees at 6 V by the vendor, scaled to the Hub's 5 V:"
                + " 0.168 s per 60 degrees, and held at its angle whatever the load";
        static final double SECONDS_PER_SIXTY_DEGREES = 0.168;
        static final double SIXTY_DEGREES = 60;
        static final String RANGE_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the ramp servo's speed is set"
                + " from 0.05 to 1 s per 60 degrees";
        static final double LEAST_SECONDS_PER_SIXTY_DEGREES = 0.05;
        static final double MOST_SECONDS_PER_SIXTY_DEGREES = 1;

        private final double secondsPerSixtyDegrees;

        private Ramp(double secondsPerSixtyDegrees) {
            this.secondsPerSixtyDegrees = secondsPerSixtyDegrees;
        }

        public static Checked<Ramp> of(double secondsPerSixtyDegrees) {
            if (!(secondsPerSixtyDegrees >= LEAST_SECONDS_PER_SIXTY_DEGREES
                    && secondsPerSixtyDegrees <= MOST_SECONDS_PER_SIXTY_DEGREES)) {
                return Checked.rejected("a ramp servo's speed is from " + LEAST_SECONDS_PER_SIXTY_DEGREES + " to "
                        + MOST_SECONDS_PER_SIXTY_DEGREES + " s per 60 degrees, not " + secondsPerSixtyDegrees);
            }
            return Checked.ok(new Ramp(secondsPerSixtyDegrees));
        }

        public static Ramp revSmartRobotServo() {
            return new Ramp(SECONDS_PER_SIXTY_DEGREES);
        }

        public RampAngle next(RampAngle from, RampAngle commanded, Torque load, Seconds step) {
            double most = SIXTY_DEGREES * step.value() / secondsPerSixtyDegrees;
            double toGo = commanded.degrees - from.degrees;
            return StrictMath.abs(toGo) <= most
                    ? commanded
                    : new RampAngle(from.degrees + StrictMath.copySign(most, toGo));
        }

        @Override
        public String toString() {
            return "a ramp servo slewing " + secondsPerSixtyDegrees + " s per 60 degrees";
        }
    }

    public static final class RampAngle {
        static final String RANGE_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the ramp is set from 0 to 60"
                + " degrees, raised from where the CAD draws it";
        static final double MOST_DEGREES = 60;

        private final double degrees;

        private RampAngle(double degrees) {
            this.degrees = degrees;
        }

        public static Checked<RampAngle> ofDegrees(double degrees) {
            if (!(degrees >= 0 && degrees <= MOST_DEGREES)) {
                return Checked.rejected("a ramp angle is from 0 to " + MOST_DEGREES
                        + " degrees above where it is drawn, not " + degrees);
            }
            return Checked.ok(new RampAngle(degrees));
        }

        public double degrees() {
            return degrees;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof RampAngle that && Double.compare(degrees, that.degrees) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(degrees);
        }

        @Override
        public String toString() {
            return degrees + " degrees";
        }
    }

    public static final class Room {
        static final double FLOOR_TOP_MM = 15;
        static final double WALL_RADIUS_MM = 82.27;
        static final double PLATE_UNDERSIDE_MM = 96.12;

        private final double floorTop;
        private final double wallRadius;
        private final double plateUnderside;

        private Room(double floorTop, double wallRadius, double plateUnderside) {
            this.floorTop = floorTop;
            this.wallRadius = wallRadius;
            this.plateUnderside = plateUnderside;
        }

        public static Checked<Room> of(double floorTopMm, double wallRadiusMm, double plateUndersideMm) {
            if (!finite(floorTopMm, wallRadiusMm, plateUndersideMm)) {
                return Checked.rejected("a feeder's room is measured in finite mm");
            }
            if (!(wallRadiusMm > 0)) {
                return Checked.rejected("a feeder's wall has a positive radius, not " + wallRadiusMm);
            }
            if (!(plateUndersideMm > floorTopMm)) {
                return Checked.rejected("a feeder's top plate is above its floor, not at " + plateUndersideMm
                        + " over a floor at " + floorTopMm);
            }
            return Checked.ok(new Room(floorTopMm, wallRadiusMm, plateUndersideMm));
        }
    }

    public static final class Arms {
        static final String SHAPE_SOURCE = "doc/plans/feeder-sim.md, Questions for the team, 2: not drawn, so the"
                + " plan's for now, a hub of r 10 and two arms opposite, 4 thick, from the hub to r 50, from 2 above"
                + " the floor to z 80";
        static final double HUB_RADIUS_MM = 10;
        static final double ARM_THICKNESS_MM = 4;
        static final double ARM_REACH_MM = 50;
        static final double ARM_BOTTOM_MM = 17;
        static final double ARM_TOP_MM = 80;

        private final double hubRadius;
        private final double armThickness;
        private final double armReach;
        private final double armBottom;
        private final double armTop;

        private Arms(double hubRadius, double armThickness, double armReach, double armBottom, double armTop) {
            this.hubRadius = hubRadius;
            this.armThickness = armThickness;
            this.armReach = armReach;
            this.armBottom = armBottom;
            this.armTop = armTop;
        }

        public static Checked<Arms> of(
                double hubRadiusMm, double armThicknessMm, double armReachMm, double armBottomMm, double armTopMm) {
            if (!finite(hubRadiusMm, armThicknessMm, armReachMm, armBottomMm, armTopMm)) {
                return Checked.rejected("a spinner is measured in finite mm");
            }
            if (!(hubRadiusMm > 0 && armThicknessMm > 0)) {
                return Checked.rejected("a spinner's hub and arms have a positive size, not a hub of r " + hubRadiusMm
                        + " and arms " + armThicknessMm + " thick");
            }
            if (!(armReachMm > hubRadiusMm)) {
                return Checked.rejected(
                        "a spinner's arms reach past its hub of r " + hubRadiusMm + ", not to r " + armReachMm);
            }
            if (!(armTopMm > armBottomMm)) {
                return Checked.rejected("a spinner's arms stand higher at their top than their bottom, not from z "
                        + armBottomMm + " to z " + armTopMm);
            }
            return Checked.ok(new Arms(hubRadiusMm, armThicknessMm, armReachMm, armBottomMm, armTopMm));
        }

        @Override
        public String toString() {
            return "a spinner with a hub of r " + hubRadius + " and arms " + armThickness + " thick out to r "
                    + armReach + ", from z " + armBottom + " to z " + armTop;
        }
    }

    public static final class Bar {
        static final String SHAPE_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: Part 2 runs side to side, 5"
                + " tall with its top at z -11.49 as drawn, at y about -54, from x -31.96 to the pad at x 38, swung"
                + " about the ramp servo's shaft along y at x 57.54, z -13.99; How it is simulated: 6 wide, its top"
                + " edges rounded 1, inferred, so centred in the slot's 7 at y -53.95";
        static final double PIVOT_X_MM = 57.54;
        static final double PIVOT_Z_MM = -13.99;
        static final double TOP_AS_DRAWN_MM = -11.49;
        static final double HEIGHT_MM = 5;
        static final double WIDTH_MM = 6;
        static final double ROUNDING_MM = 1;
        static final double MIDDLE_Y_MM = -53.95;
        static final double FREE_END_X_MM = -31.96;
        static final double PAD_X_MM = 38;

        private final double pivotX;
        private final double pivotZ;
        private final double topAsDrawn;
        private final double height;
        private final double width;
        private final double rounding;
        private final double middleY;
        private final double freeEndX;
        private final double padX;

        private Bar(
                double pivotX,
                double pivotZ,
                double topAsDrawn,
                double height,
                double width,
                double rounding,
                double middleY,
                double freeEndX,
                double padX) {
            this.pivotX = pivotX;
            this.pivotZ = pivotZ;
            this.topAsDrawn = topAsDrawn;
            this.height = height;
            this.width = width;
            this.rounding = rounding;
            this.middleY = middleY;
            this.freeEndX = freeEndX;
            this.padX = padX;
        }

        public static Checked<Bar> of(
                double pivotXMm,
                double pivotZMm,
                double topAsDrawnMm,
                double heightMm,
                double widthMm,
                double roundingMm,
                double middleYMm,
                double freeEndXMm,
                double padXMm) {
            if (!finite(
                    pivotXMm, pivotZMm, topAsDrawnMm, heightMm, widthMm, roundingMm, middleYMm, freeEndXMm, padXMm)) {
                return Checked.rejected("a ramp bar is measured in finite mm");
            }
            if (!(heightMm > 0 && widthMm > 0)) {
                return Checked.rejected(
                        "a ramp bar has a positive height and width, not " + heightMm + " by " + widthMm);
            }
            if (!(roundingMm >= 0 && roundingMm <= heightMm && 2 * roundingMm <= widthMm)) {
                return Checked.rejected("a ramp bar's top edges are rounded by no more than its height and half its"
                        + " width, not " + roundingMm);
            }
            if (!(freeEndXMm < padXMm && padXMm < pivotXMm)) {
                return Checked.rejected("a ramp bar runs from its free end to its pad, toward its pivot, not from x "
                        + freeEndXMm + " to x " + padXMm + " with the pivot at x " + pivotXMm);
            }
            return Checked.ok(new Bar(
                    pivotXMm, pivotZMm, topAsDrawnMm, heightMm, widthMm, roundingMm, middleYMm, freeEndXMm, padXMm));
        }

        @Override
        public String toString() {
            return "a ramp bar " + height + " tall and " + width + " wide, rounded " + rounding + ", at y " + middleY
                    + " from x " + freeEndX + " to x " + padX + ", its top at z " + topAsDrawn
                    + " as drawn, swung about"
                    + " (x " + pivotX + ", z " + pivotZ + ")";
        }

        private Vec3 raised(double drawnX, double drawnZ, RampAngle by) {
            double turn = StrictMath.toRadians(by.degrees());
            double cos = StrictMath.cos(turn);
            double sin = StrictMath.sin(turn);
            double out = drawnX - pivotX;
            double up = drawnZ - pivotZ;
            return new Vec3(pivotX + out * cos + up * sin, middleY, pivotZ + up * cos - out * sin);
        }

        private double highestResting(Vec3 ball, double ballRadius, RampAngle by) {
            double turn = StrictMath.toRadians(by.degrees());
            Vec3 along = new Vec3(StrictMath.cos(turn), 0, -StrictMath.sin(turn));
            Vec3 up = new Vec3(StrictMath.sin(turn), 0, StrictMath.cos(turn));
            double length = padX - freeEndX;
            double edgeFromMiddle = width / 2 - rounding;
            double highest = Double.NEGATIVE_INFINITY;
            Vec3 topEnd = raised(freeEndX, topAsDrawn, by);
            if (StrictMath.abs(ball.y() - middleY) <= edgeFromMiddle) {
                highest = StrictMath.max(highest, onTheLine(ball, topEnd, along, up, length, ballRadius));
            }
            for (double side = -1; side <= 1; side += 2) {
                Vec3 end = raised(freeEndX, topAsDrawn - rounding, by).plus(new Vec3(0, side * edgeFromMiddle, 0));
                double reach = ballRadius + rounding;
                double across = ball.y() - end.y();
                double inPlane = reach * reach - across * across;
                if (inPlane >= 0) {
                    highest =
                            StrictMath.max(highest, onTheLine(ball, end, along, up, length, StrictMath.sqrt(inPlane)));
                    highest = StrictMath.max(highest, onTheEnd(ball, end, reach));
                    highest = StrictMath.max(highest, onTheEnd(ball, end.along(along, length), reach));
                }
            }
            return highest;
        }

        private static double onTheLine(Vec3 ball, Vec3 from, Vec3 along, Vec3 up, double length, double above) {
            double z = from.z() + (above - (ball.x() - from.x()) * up.x()) / up.z();
            double at = new Vec3(ball.x(), from.y(), z).minus(from).dot(along);
            return at >= 0 && at <= length ? z : Double.NEGATIVE_INFINITY;
        }

        private static double onTheEnd(Vec3 ball, Vec3 end, double reach) {
            double dx = ball.x() - end.x();
            double dy = ball.y() - end.y();
            double inHeight = reach * reach - dx * dx - dy * dy;
            return inHeight >= 0 ? end.z() + StrictMath.sqrt(inHeight) : Double.NEGATIVE_INFINITY;
        }

        private double nearestToTheAxis() {
            return StrictMath.max(0, StrictMath.abs(middleY) - width / 2);
        }
    }

    public static final class Window {
        static final String CUT_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: through the wall, the front"
                + " window at |x| < 63 with its sill at z 31.12, the entry from the scoop, and the back window at"
                + " |x| < 40.6 with its sill at z 43.12";
        static final double FRONT_HALF_WIDTH_MM = 63;
        static final double FRONT_SILL_MM = 31.12;
        static final double BACK_HALF_WIDTH_MM = 40.6;
        static final double BACK_SILL_MM = 43.12;

        private final double halfWidth;
        private final double sill;

        private Window(double halfWidth, double sill) {
            this.halfWidth = halfWidth;
            this.sill = sill;
        }

        public static Checked<Window> of(double halfWidthMm, double sillMm) {
            if (!finite(halfWidthMm, sillMm)) {
                return Checked.rejected("a window is measured in finite mm");
            }
            if (!(halfWidthMm > 0)) {
                return Checked.rejected("a window has a positive width, not half of it " + halfWidthMm);
            }
            return Checked.ok(new Window(halfWidthMm, sillMm));
        }

        private Checked<Window> through(Room room) {
            if (!(sill > room.floorTop && sill < room.plateUnderside)) {
                return Checked.rejected("a window's sill is above the floor at z " + room.floorTop
                        + " and below the top plate at z " + room.plateUnderside + ", not at z " + sill);
            }
            if (!(halfWidth < room.wallRadius)) {
                return Checked.rejected("a window is narrower than the wall of r " + room.wallRadius
                        + " it is cut through, not " + 2 * halfWidth + " wide");
            }
            return Checked.ok(this);
        }

        @Override
        public String toString() {
            return "a window at |x| < " + halfWidth + " over a sill at z " + sill;
        }
    }

    public static final class Plate {
        static final String CUT_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: FeederTop, cut flat at y -78.78"
                + " and y 62.28, with a notch at |x| < 31.739 from y -26.843 out through the back, which lets the"
                + " flywheel reach into the feeder";
        static final double BACK_FLAT_Y_MM = -78.78;
        static final double FRONT_FLAT_Y_MM = 62.28;
        static final double NOTCH_HALF_WIDTH_MM = 31.739;
        static final double NOTCH_INNER_END_Y_MM = -26.843;

        private final double backFlatY;
        private final double frontFlatY;
        private final double notchHalfWidth;
        private final double notchInnerEndY;

        private Plate(double backFlatY, double frontFlatY, double notchHalfWidth, double notchInnerEndY) {
            this.backFlatY = backFlatY;
            this.frontFlatY = frontFlatY;
            this.notchHalfWidth = notchHalfWidth;
            this.notchInnerEndY = notchInnerEndY;
        }

        public static Checked<Plate> of(
                double backFlatYMm, double frontFlatYMm, double notchHalfWidthMm, double notchInnerEndYMm) {
            if (!finite(backFlatYMm, frontFlatYMm, notchHalfWidthMm, notchInnerEndYMm)) {
                return Checked.rejected("a top plate is measured in finite mm");
            }
            if (!(backFlatYMm < frontFlatYMm)) {
                return Checked.rejected("a top plate's back flat is behind its front flat, not at y " + backFlatYMm
                        + " against y " + frontFlatYMm);
            }
            if (!(notchHalfWidthMm > 0)) {
                return Checked.rejected("a top plate's notch has a positive width, not half of it " + notchHalfWidthMm);
            }
            return Checked.ok(new Plate(backFlatYMm, frontFlatYMm, notchHalfWidthMm, notchInnerEndYMm));
        }

        private Checked<Plate> over(Room room) {
            if (!(notchInnerEndY > backFlatY && notchInnerEndY < frontFlatY)) {
                return Checked.rejected("a top plate's notch ends inside the plate, between y " + backFlatY + " and y "
                        + frontFlatY + ", not at y " + notchInnerEndY);
            }
            if (!(notchHalfWidth < room.wallRadius)) {
                return Checked.rejected("a top plate's notch is narrower than the wall of r " + room.wallRadius
                        + ", not " + 2 * notchHalfWidth + " wide");
            }
            return Checked.ok(this);
        }

        @Override
        public String toString() {
            return "a top plate cut flat at y " + backFlatY + " and y " + frontFlatY + ", notched at |x| < "
                    + notchHalfWidth + " behind y " + notchInnerEndY;
        }
    }

    public static final class Slot {
        static final String CUT_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: the floor slot, through the"
                + " floor, x -32.17 to 47.83, y -57.45 to -50.45";
        static final double FROM_X_MM = -32.17;
        static final double TO_X_MM = 47.83;
        static final double FROM_Y_MM = -57.45;
        static final double TO_Y_MM = -50.45;

        private final double fromX;
        private final double toX;
        private final double fromY;
        private final double toY;

        private Slot(double fromX, double toX, double fromY, double toY) {
            this.fromX = fromX;
            this.toX = toX;
            this.fromY = fromY;
            this.toY = toY;
        }

        public static Checked<Slot> of(double fromXMm, double toXMm, double fromYMm, double toYMm) {
            if (!finite(fromXMm, toXMm, fromYMm, toYMm)) {
                return Checked.rejected("a floor slot is measured in finite mm");
            }
            if (!(fromXMm < toXMm && fromYMm < toYMm)) {
                return Checked.rejected("a floor slot runs from its lesser x and y to its greater, not from (" + fromXMm
                        + ", " + fromYMm + ") to (" + toXMm + ", " + toYMm + ")");
            }
            return Checked.ok(new Slot(fromXMm, toXMm, fromYMm, toYMm));
        }

        private boolean fits(Bar bar) {
            return fromY <= bar.middleY - bar.width / 2
                    && bar.middleY + bar.width / 2 <= toY
                    && fromX <= bar.freeEndX
                    && bar.padX <= toX;
        }

        @Override
        public String toString() {
            return "a floor slot from (" + fromX + ", " + fromY + ") to (" + toX + ", " + toY + ")";
        }
    }

    public static final class Wheel {
        static final String PLACE_SOURCE = "doc/plans/feeder-sim.md, What the CAD says: FlywheelOuter's axle runs"
                + " along x through feeder (y -84.13, z 132.82), its rim the groove Launcher models";
        static final double AXLE_Y_MM = -84.13;
        static final double AXLE_Z_MM = 132.82;

        private final double axleY;
        private final double axleZ;

        private Wheel(double axleY, double axleZ) {
            this.axleY = axleY;
            this.axleZ = axleZ;
        }

        public static Checked<Wheel> of(double axleYMm, double axleZMm) {
            if (!finite(axleYMm, axleZMm)) {
                return Checked.rejected("a flywheel's axle is placed in finite mm");
            }
            return Checked.ok(new Wheel(axleYMm, axleZMm));
        }

        private double lowest() {
            return axleZ - Launcher.FLANGE_MM;
        }

        private double distanceTo(Vec3 point) {
            double across = StrictMath.abs(point.x());
            double out = StrictMath.hypot(point.y() - axleY, point.z() - axleZ);
            double halfWidth = Launcher.RINGS_EACH_SIDE * Launcher.RING_APART_MM;
            double toTheFlange = StrictMath.hypot(across - halfWidth, out - Launcher.FLANGE_MM);
            if (across > halfWidth) {
                return out <= Launcher.FLANGE_MM ? across - halfWidth : toTheFlange;
            }
            double fromTheTube = StrictMath.hypot(across, out - Launcher.WHEEL_TORUS_MM);
            boolean facingTheGroove =
                    out < Launcher.WHEEL_TORUS_MM && across * Launcher.WHEEL_TUBE_MM <= halfWidth * fromTheTube;
            return facingTheGroove ? Launcher.WHEEL_TUBE_MM - fromTheTube : toTheFlange;
        }

        @Override
        public String toString() {
            return "a flywheel on an axle along x at (y " + axleY + ", z " + axleZ + ")";
        }
    }

    public static final class Mount {
        static final String PLACE_SOURCE = "doc/plans/feeder-sim.md, What the CAD says, Placed in Reginald: the feeder"
                + " frame maps to the Reginald part studio (mm; +X left, +Y back, +Z up, origin at the chassis's front"
                + " middle at floor level) from (0, 173.96, 28.78), tilted 16.46 degrees, rising toward the back";
        static final double ORIGIN_BACK_MM = 173.96;
        static final double ORIGIN_UP_MM = 28.78;
        static final double TILT_DEGREES = 16.46;

        private final Vec3 origin;
        private final Tilt tilt;

        private Mount(Vec3 origin, Tilt tilt) {
            this.origin = origin;
            this.tilt = tilt;
        }

        public static Checked<Mount> of(Vec3 originMm, Tilt tilt) {
            if (!originMm.isFinite()) {
                return Checked.rejected("a feeder is mounted at a finite place in Reginald, not " + originMm);
            }
            return Checked.ok(new Mount(originMm, tilt));
        }

        private Vec3 inReginald(Vec3 feeder) {
            double turn = StrictMath.toRadians(tilt.degrees());
            double cos = StrictMath.cos(turn);
            double sin = StrictMath.sin(turn);
            return origin.plus(
                    new Vec3(-feeder.x(), -feeder.y() * cos - feeder.z() * sin, -feeder.y() * sin + feeder.z() * cos));
        }

        @Override
        public String toString() {
            return "a feeder mounted at " + origin + " in Reginald, tilted " + tilt;
        }
    }

    public static final class Station {
        private final Shape shape;
        private final double radius;
        private final Vec3 resting;

        private Station(Shape shape, double radius, Vec3 resting) {
            this.shape = shape;
            this.radius = radius;
            this.resting = resting;
        }

        public double liftMm(RampAngle raised) {
            return StrictMath.max(0, shape.bar.highestResting(resting, radius, raised) - resting.z());
        }

        public double wheelClearanceMm(RampAngle raised) {
            return shape.wheel.distanceTo(resting.plus(new Vec3(0, 0, liftMm(raised)))) - radius;
        }
    }

    public static final class Seat {
        private final double hubClearance;
        private final double wallClearance;
        private final double armClearance;

        private Seat(double hubClearance, double wallClearance, double armClearance) {
            this.hubClearance = hubClearance;
            this.wallClearance = wallClearance;
            this.armClearance = armClearance;
        }

        public double hubClearanceMm() {
            return hubClearance;
        }

        public double wallClearanceMm() {
            return wallClearance;
        }

        public double armClearanceMm() {
            return armClearance;
        }
    }

    public static final class Shape {
        private final Room room;
        private final Window front;
        private final Window back;
        private final Plate plate;
        private final Slot slot;
        private final Arms arms;
        private final Bar bar;
        private final Wheel wheel;
        private final Mount mount;

        private Shape(
                Room room,
                Window front,
                Window back,
                Plate plate,
                Slot slot,
                Arms arms,
                Bar bar,
                Wheel wheel,
                Mount mount) {
            this.room = room;
            this.front = front;
            this.back = back;
            this.plate = plate;
            this.slot = slot;
            this.arms = arms;
            this.bar = bar;
            this.wheel = wheel;
            this.mount = mount;
        }

        public static Checked<Shape> of(
                Room room,
                Window front,
                Window back,
                Plate plate,
                Slot slot,
                Arms arms,
                Bar bar,
                Wheel wheel,
                Mount mount) {
            return front.through(room)
                    .then(cut -> back.through(room))
                    .then(cut -> plate.over(room))
                    .then(cut -> fitted(room, front, back, plate, slot, arms, bar, wheel, mount));
        }

        private static Checked<Shape> fitted(
                Room room,
                Window front,
                Window back,
                Plate plate,
                Slot slot,
                Arms arms,
                Bar bar,
                Wheel wheel,
                Mount mount) {
            if (!slot.fits(bar)) {
                return Checked.rejected("a ramp bar fits " + slot + " through the floor; not " + bar);
            }
            if (!(arms.armBottom > room.floorTop)) {
                return Checked.rejected(
                        "a spinner's arms stand above the floor at z " + room.floorTop + "; not " + arms);
            }
            if (!(arms.armTop < wheel.lowest())) {
                return Checked.rejected("a spinner's arms stand under the wheel, whose lowest point is at z "
                        + wheel.lowest() + "; not " + arms);
            }
            if (!(arms.armReach < bar.nearestToTheAxis())) {
                return Checked.rejected("a spinner's arms stop short of the raised bar, r " + bar.nearestToTheAxis()
                        + " from the axis, unless cut away above it; not " + arms);
            }
            return Checked.ok(new Shape(room, front, back, plate, slot, arms, bar, wheel, mount));
        }

        public static Checked<Shape> cad() {
            return Room.of(Room.FLOOR_TOP_MM, Room.WALL_RADIUS_MM, Room.PLATE_UNDERSIDE_MM)
                    .then(room -> Window.of(Window.FRONT_HALF_WIDTH_MM, Window.FRONT_SILL_MM)
                            .then(front -> Window.of(Window.BACK_HALF_WIDTH_MM, Window.BACK_SILL_MM)
                                    .then(back -> Plate.of(
                                                    Plate.BACK_FLAT_Y_MM,
                                                    Plate.FRONT_FLAT_Y_MM,
                                                    Plate.NOTCH_HALF_WIDTH_MM,
                                                    Plate.NOTCH_INNER_END_Y_MM)
                                            .then(plate -> Slot.of(
                                                            Slot.FROM_X_MM, Slot.TO_X_MM, Slot.FROM_Y_MM, Slot.TO_Y_MM)
                                                    .then(slot -> cad(room, front, back, plate, slot))))));
        }

        private static Checked<Shape> cad(Room room, Window front, Window back, Plate plate, Slot slot) {
            return Arms.of(
                            Arms.HUB_RADIUS_MM,
                            Arms.ARM_THICKNESS_MM,
                            Arms.ARM_REACH_MM,
                            Arms.ARM_BOTTOM_MM,
                            Arms.ARM_TOP_MM)
                    .then(arms -> Bar.of(
                                    Bar.PIVOT_X_MM,
                                    Bar.PIVOT_Z_MM,
                                    Bar.TOP_AS_DRAWN_MM,
                                    Bar.HEIGHT_MM,
                                    Bar.WIDTH_MM,
                                    Bar.ROUNDING_MM,
                                    Bar.MIDDLE_Y_MM,
                                    Bar.FREE_END_X_MM,
                                    Bar.PAD_X_MM)
                            .then(bar -> Wheel.of(Wheel.AXLE_Y_MM, Wheel.AXLE_Z_MM)
                                    .then(wheel -> Tilt.of(Mount.TILT_DEGREES)
                                            .then(tilt -> Mount.of(
                                                    new Vec3(0, Mount.ORIGIN_BACK_MM, Mount.ORIGIN_UP_MM), tilt))
                                            .then(mount ->
                                                    of(room, front, back, plate, slot, arms, bar, wheel, mount)))));
        }

        public Checked<Shape> with(Arms other) {
            return of(room, front, back, plate, slot, other, bar, wheel, mount);
        }

        public Checked<Shape> withFrontWindow(Window other) {
            return of(room, other, back, plate, slot, arms, bar, wheel, mount);
        }

        public Checked<Shape> withBackWindow(Window other) {
            return of(room, front, other, plate, slot, arms, bar, wheel, mount);
        }

        public Checked<Shape> with(Plate other) {
            return of(room, front, back, other, slot, arms, bar, wheel, mount);
        }

        public Checked<Shape> with(Bar other) {
            return of(room, front, back, plate, slot, arms, other, wheel, mount);
        }

        public Checked<Double> barTopMm(double drawnXMm, RampAngle raised) {
            if (!(drawnXMm >= bar.freeEndX && drawnXMm <= bar.padX)) {
                return Checked.rejected("a point on the ramp bar is drawn from its free end at x " + bar.freeEndX
                        + " to its pad at x " + bar.padX + ", not at x " + drawnXMm);
            }
            return Checked.ok(bar.raised(drawnXMm, bar.topAsDrawn, raised).z());
        }

        public Vec3 inReginaldMm(Vec3 feederMm) {
            return mount.inReginald(feederMm);
        }

        public Checked<Station> atTheBackStation(Length pollen, double xMm) {
            double radius = pollen.inches() * Launcher.MILLIMETRES_PER_INCH;
            double centre = room.wallRadius - radius;
            if (!(StrictMath.abs(xMm) < centre)) {
                return Checked.rejected("a pollen at the back station against the wall is centred within " + centre
                        + " of the axis across, not at x " + xMm);
            }
            return Checked.ok(new Station(
                    this,
                    radius,
                    new Vec3(xMm, -StrictMath.sqrt(centre * centre - xMm * xMm), room.floorTop + radius)));
        }

        public Checked<Seat> seatedAgainstTheWall(Length pollen) {
            double radius = pollen.inches() * Launcher.MILLIMETRES_PER_INCH;
            double centre = room.wallRadius - radius;
            double hubClearance = centre - radius - arms.hubRadius;
            if (!(hubClearance >= 0)) {
                return Checked.rejected("a pollen seated against the wall clears the spinner's hub, not a pollen of r "
                        + radius + " mm, which overlaps it by " + -hubClearance);
            }
            return Checked.ok(
                    new Seat(hubClearance, room.wallRadius - centre - radius, centre - radius - arms.armThickness / 2));
        }

        public double floorTopMm() {
            return room.floorTop;
        }

        public double wallRadiusMm() {
            return room.wallRadius;
        }

        public double roomMm() {
            return room.plateUnderside - room.floorTop;
        }
    }
}
