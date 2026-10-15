package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public final class Launcher implements Surroundings.Part {
    static final String CAD_SOURCE =
            "Reginald's CAD, the 13 October snapshot's exact faces, with the hood and the lead-in"
                    + " as the team allowed them changed: doc/plans/runtime-flight.md, stage 4, What the CAD says and What we"
                    + " change";
    static final double MILLIMETRES_PER_INCH = 25.4;
    static final double AXLE_BEHIND_CENTRE_MM = 217 - 204;
    static final double AXLE_UP_MM = 180;
    static final double WHEEL_TORUS_MM = 75.2886;
    static final double WHEEL_TUBE_MM = 35.56;
    static final double HOOD_TORUS_MM = 72.341;
    static final double HOOD_TUBE_MM = 36;
    static final double RING_APART_MM = 12.5;
    static final int RINGS_EACH_SIDE = 2;
    static final int RINGS = 2 * RINGS_EACH_SIDE + 1;
    static final double HOOD_FROM_DEGREES = -64.8;
    static final double HOOD_TO_DEGREES = 20;
    static final double EXTENSION_FROM_DEGREES = -73;
    static final String POLLEN_SOURCE = "a flight's pollen is 1.39 in in radius, as the field's pieces are: the"
            + " simulated 70.61 mm pollen the plan's hood squeezes by 2.0 mm";
    static final double POLLEN_RADIUS_IN = 1.39;
    static final double LEAD_IN_SQUEEZE_MM = 0.25;
    static final double GROOVE_BOTTOM_MM = WHEEL_TORUS_MM - WHEEL_TUBE_MM;
    static final double FLANGE_MM = WHEEL_TORUS_MM
            - StrictMath.sqrt(
                    WHEEL_TUBE_MM * WHEEL_TUBE_MM - RINGS_EACH_SIDE * RING_APART_MM * RINGS_EACH_SIDE * RING_APART_MM);
    static final double LEAD_IN_LOWER_END_MM =
            GROOVE_BOTTOM_MM + 2 * POLLEN_RADIUS_IN * MILLIMETRES_PER_INCH - LEAD_IN_SQUEEZE_MM;
    static final double INNERMOST_MM = GROOVE_BOTTOM_MM;
    static final double OUTERMOST_MM = StrictMath.hypot(LEAD_IN_LOWER_END_MM, POLLEN_RADIUS_IN * MILLIMETRES_PER_INCH);
    static final double WIDEST_MM = HOOD_TUBE_MM;
    static final double FARTHEST_IN =
            StrictMath.hypot(AXLE_BEHIND_CENTRE_MM + OUTERMOST_MM, WIDEST_MM) / MILLIMETRES_PER_INCH;
    static final String STOP = "Launcher stop";
    static final String STOP_SOURCE = "a rigid face across the path at the hood extension's lower end, as the frame's"
            + " metal is rigid: it stands one pollen's radius below the entry, from the flange out to the lead-in's"
            + " lower end and across the hood's groove, so a fed ball rests on it while the wheel is stopped";
    static final String FEED_SOURCE = "0.1 m/s along the path at the lead-in's lower end stands in for the ramp that"
            + " lifts a pollen into the flywheel, which doc/plans/reginald-internals.md simulates";
    static final double FEED_M_PER_S = 0.1;

    record Placed(Vec3 origin, double yaw) {}

    public static final class Frame {
        static final String PLACE_SOURCE = "a robot is placed within the field's walls, under 72 in from the centre of"
                + " the 141.17 in field (Placement): within 10000 in of it either way every pose a run gives is a"
                + " frame, and a frame carried at the most a course sweeps for as long as a grid time counts stays"
                + " finite";
        static final double MOST_FROM_CENTRE_IN = 10000;

        private final double x;
        private final double y;
        private final double yaw;

        private Frame(double x, double y, double yaw) {
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }

        public static Checked<Frame> of(double x, double y, double yaw) {
            if (!(StrictMath.abs(x) <= MOST_FROM_CENTRE_IN && StrictMath.abs(y) <= MOST_FROM_CENTRE_IN)) {
                return Checked.rejected("a launcher's frame is within " + MOST_FROM_CENTRE_IN
                        + " in of the field's centre either way, not at (" + x + ", " + y + ")");
            }
            if (!Double.isFinite(yaw)) {
                return Checked.rejected("a launcher's frame faces a finite yaw, not " + yaw);
            }
            return Checked.ok(new Frame(
                    withoutNegativeZero(x),
                    withoutNegativeZero(y),
                    withoutNegativeZero(StrictMath.IEEEremainder(yaw, Flywheel.WHOLE_TURN))));
        }

        private static double withoutNegativeZero(double value) {
            return value + 0.0;
        }

        double x() {
            return x;
        }

        double y() {
            return y;
        }

        double yaw() {
            return yaw;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Frame that
                    && Double.compare(x, that.x) == 0
                    && Double.compare(y, that.y) == 0
                    && Double.compare(yaw, that.yaw) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y, yaw);
        }

        @Override
        public String toString() {
            return "(" + x + ", " + y + ") facing " + yaw;
        }
    }

    static final class Course {
        static final String SWEEP_SOURCE = "a launcher's faces carry the balls in it, so none moves faster than a"
                + " flight takes a ball, Flight.MOST_SPEED_IN_PER_S: the frame's speed and its turn times the"
                + " farthest any face is from the axis it turns about";

        private final Frame from;
        private final GridTime began;
        private final Frame to;
        private final GridTime ends;
        private final Vec3 velocity;
        private final double turning;

        private Course(Frame from, GridTime began, Frame to, GridTime ends, Vec3 velocity, double turning) {
            this.from = from;
            this.began = began;
            this.to = to;
            this.ends = ends;
            this.velocity = velocity;
            this.turning = turning;
        }

        static Checked<Course> of(Frame from, GridTime began, Frame to, GridTime ends) {
            double seconds = between(began, ends);
            if (!(seconds >= 0)) {
                return Checked.rejected(
                        "a launcher's course ends no earlier than it begins, not at " + ends + " from " + began);
            }
            if (!(seconds > 0)) {
                return Checked.ok(new Course(from, began, to, ends, Vec3.zero(), 0));
            }
            Course course = new Course(
                    from,
                    began,
                    to,
                    ends,
                    new Vec3((to.x - from.x) / seconds, (to.y - from.y) / seconds, 0),
                    StrictMath.IEEEremainder(to.yaw - from.yaw, Flywheel.WHOLE_TURN) / seconds);
            double sweep = course.sweep();
            if (!(sweep <= Flight.MOST_SPEED_IN_PER_S)) {
                return Checked.rejected("a launcher's frame sweeps its faces at most " + Flight.MOST_SPEED_IN_PER_S
                        + " in/s, as fast as a flight takes a ball, not " + sweep + " in/s, going at "
                        + course.velocity() + " in/s and turning at " + course.turning() + " rad/s");
            }
            return Checked.ok(course);
        }

        private static double between(GridTime earlier, GridTime later) {
            return later.seconds() - earlier.seconds();
        }

        Course movingAsItWasAt(Frame at) {
            return new Course(at, ends, at, ends, velocity, turning);
        }

        GridTime began() {
            return began;
        }

        Frame endsAt() {
            return to;
        }

        GridTime ends() {
            return ends;
        }

        Vec3 velocity() {
            return velocity;
        }

        double turning() {
            return turning;
        }

        double sweep() {
            return StrictMath.sqrt(velocity.dot(velocity)) + StrictMath.abs(turning) * FARTHEST_IN;
        }

        boolean moving() {
            return velocity.x() != 0 || velocity.y() != 0 || turning != 0;
        }

        Placed at(GridTime time) {
            if (!moving()) {
                return new Placed(new Vec3(to.x, to.y, 0), to.yaw);
            }
            double sinceBegan = between(began, time);
            double sinceEnded = between(ends, time);
            if (StrictMath.abs(sinceBegan) < StrictMath.abs(sinceEnded)) {
                return carried(from, sinceBegan);
            }
            return carried(to, sinceEnded);
        }

        private Placed carried(Frame frame, double seconds) {
            return new Placed(
                    new Vec3(frame.x + seconds * velocity.x(), frame.y + seconds * velocity.y(), 0),
                    frame.yaw + seconds * turning);
        }

        @Override
        public String toString() {
            return "from " + from + " at " + began + " to " + to + " " + between(began, ends) + " s later";
        }
    }

    private record WheelRing(String id, Arc arc) {}

    private record HoodRing(String id, Arc main, Arc leadIn) {
        Arc.Nearest nearestTo(Vec3 local) {
            Arc.Nearest onTheHood = main.nearestTo(local);
            Arc.Nearest onTheLeadIn = leadIn.nearestTo(local);
            return onTheLeadIn.distance() < onTheHood.distance() ? onTheLeadIn : onTheHood;
        }
    }

    private record Way(double along, double up) {
        static Way at(double radians, double out) {
            return new Way(out * StrictMath.cos(radians), out * StrictMath.sin(radians));
        }

        Way minus(Way other) {
            return new Way(along - other.along, up - other.up);
        }

        double length() {
            return StrictMath.hypot(along, up);
        }

        double angle() {
            return StrictMath.atan2(up, along);
        }

        Vec3 local() {
            return new Vec3(-along, 0, up);
        }
    }

    static final class Faces {
        private final List<WheelRing> wheel;
        private final List<HoodRing> hood;
        private final Collider stop;
        private final Compliance wheelSurface;
        private final Compliance hoodSurface;

        private Faces(
                List<WheelRing> wheel,
                List<HoodRing> hood,
                Collider stop,
                Compliance wheelSurface,
                Compliance hoodSurface) {
            this.wheel = wheel;
            this.hood = hood;
            this.stop = stop;
            this.wheelSurface = wheelSurface;
            this.hoodSurface = hoodSurface;
        }

        static Checked<Faces> built() {
            return Compliance.of(
                            Compliance.STIFFNESS_N_PER_MM,
                            Compliance.RESTITUTION,
                            Compliance.WHEEL_FRICTION,
                            Compliance.REGULARISATION_M_PER_S)
                    .then(wheel -> Compliance.of(
                                    Compliance.STIFFNESS_N_PER_MM,
                                    Compliance.RESTITUTION,
                                    Compliance.HOOD_FRICTION,
                                    Compliance.REGULARISATION_M_PER_S)
                            .then(hood -> of(wheel, hood)));
        }

        static Checked<Faces> of(Compliance wheel, Compliance hood) {
            return wheel.sharedBy(RINGS)
                    .then(wheelSurface -> hood.sharedBy(RINGS)
                            .then(hoodSurface -> wheelRings()
                                    .then(wheelRings -> hoodRings()
                                            .then(hoodRings -> stop().map(stop -> new Faces(
                                                    wheelRings, hoodRings, stop, wheelSurface, hoodSurface))))));
        }

        private static Checked<List<WheelRing>> wheelRings() {
            List<WheelRing> rings = new ArrayList<>();
            for (int ring = -RINGS_EACH_SIDE; ring <= RINGS_EACH_SIDE; ring++) {
                double across = ring * RING_APART_MM;
                Checked<Arc> arc = Arc.of(
                        ringCentre(across),
                        axle(),
                        inches(WHEEL_TORUS_MM - StrictMath.sqrt(WHEEL_TUBE_MM * WHEEL_TUBE_MM - across * across)),
                        Arc.Span.whole());
                if (arc instanceof Checked.Rejected<Arc> rejected) {
                    return Checked.rejected(rejected.rule());
                }
                arc.fold(made -> rings.add(new WheelRing("Flywheel ring at " + across + " mm", made)), rule -> false);
            }
            return Checked.ok(Collections.unmodifiableList(rings));
        }

        private static Checked<List<HoodRing>> hoodRings() {
            double opening = LEAD_IN_LOWER_END_MM - hoodRadius(0);
            double from = radians(HOOD_FROM_DEGREES);
            List<HoodRing> rings = new ArrayList<>();
            for (int ring = -RINGS_EACH_SIDE; ring <= RINGS_EACH_SIDE; ring++) {
                double across = ring * RING_APART_MM;
                double radius = hoodRadius(across);
                Way joins = Way.at(from, radius);
                Way lowerEnd = Way.at(radians(EXTENSION_FROM_DEGREES), radius + opening);
                Way centre = eccentricCentre(joins, lowerEnd, radius);
                double leadInFrom = lowerEnd.minus(centre).angle();
                double leadInWidth = joins.minus(centre).angle() - leadInFrom;
                Checked<HoodRing> made = Arc.Span.of(from, radians(HOOD_TO_DEGREES - HOOD_FROM_DEGREES))
                        .then(span -> Arc.of(ringCentre(across), axle(), inches(radius), span))
                        .then(main -> Arc.Span.of(leadInFrom, leadInWidth)
                                .then(span -> Arc.of(
                                        ringCentre(across).plus(centre.local().times(1 / MILLIMETRES_PER_INCH)),
                                        axle(),
                                        inches(radius),
                                        span))
                                .map(leadIn -> new HoodRing("Hood ring at " + across + " mm", main, leadIn)));
                if (made instanceof Checked.Rejected<HoodRing> rejected) {
                    return Checked.rejected(rejected.rule());
                }
                made.fold(hoodRing -> rings.add(hoodRing), rule -> false);
            }
            return Checked.ok(Collections.unmodifiableList(rings));
        }

        private static Way eccentricCentre(Way joins, Way lowerEnd, double radius) {
            Way chord = lowerEnd.minus(joins);
            double length = chord.length();
            double off = StrictMath.sqrt(radius * radius - length * length / 4);
            Way middle = new Way((joins.along() + lowerEnd.along()) / 2, (joins.up() + lowerEnd.up()) / 2);
            Way across = new Way(-chord.up() / length * off, chord.along() / length * off);
            Way one = new Way(middle.along() + across.along(), middle.up() + across.up());
            Way other = new Way(middle.along() - across.along(), middle.up() - across.up());
            return one.length() <= other.length() ? one : other;
        }

        private static Checked<Collider> stop() {
            double lowerEnd = radians(EXTENSION_FROM_DEGREES);
            Vec3 out = Way.at(lowerEnd, 1).local();
            Vec3 up = Way.at(lowerEnd + StrictMath.PI / 2, 1).local();
            Vec3 below = hub().minus(up.times(POLLEN_RADIUS_IN));
            Vec3 side = axle().times(inches(WIDEST_MM));
            Vec3 inner = below.plus(out.times(inches(FLANGE_MM)));
            Vec3 outer = below.plus(out.times(inches(LEAD_IN_LOWER_END_MM)));
            return Collider.plate(
                    STOP,
                    Surface.FRAME,
                    Ring.of(inner.minus(side), List.of(outer.minus(side), outer.plus(side), inner.plus(side))));
        }

        private static double hoodRadius(double across) {
            return HOOD_TORUS_MM + StrictMath.sqrt(HOOD_TUBE_MM * HOOD_TUBE_MM - across * across);
        }

        private static Vec3 ringCentre(double across) {
            return hub().plus(axle().times(inches(across)));
        }
    }

    private final Faces faces;
    private final Course course;
    private final Flywheel.AngularSpeed wheel;
    private final Placed where;
    private final double cos;
    private final double sin;
    private final Vec3 frameSpin;
    private final Checked<Surroundings.Support> wheelSupport;
    private final Checked<Surroundings.Support> hoodSupport;

    private Launcher(Faces faces, Course course, Flywheel.AngularSpeed wheel, GridTime time) {
        this.faces = faces;
        this.course = course;
        this.wheel = wheel;
        this.where = course.at(time);
        this.cos = StrictMath.cos(where.yaw());
        this.sin = StrictMath.sin(where.yaw());
        this.frameSpin = new Vec3(0, 0, course.turning());
        Vec3 hub = world(hub());
        Vec3 atHub = frameVelocityAt(hub);
        this.wheelSupport = Surroundings.Support.of(
                hub, atHub, frameSpin.plus(turned(axle()).times(wheel.radiansPerSecond())));
        this.hoodSupport = Surroundings.Support.of(hub, atHub, frameSpin);
    }

    static Launcher placed(Faces faces, Course course, Flywheel.AngularSpeed wheel) {
        return new Launcher(faces, course, wheel, course.began());
    }

    static Checked<Launcher> standing(Faces faces, Frame at, GridTime time) {
        return Course.of(at, time, at, time).map(still -> placed(faces, still, Flywheel.AngularSpeed.stopped()));
    }

    Checked<Launcher> onward(Frame to, GridTime ends, Flywheel.AngularSpeed turning) {
        return Course.of(course.endsAt(), course.ends(), to, ends).map(next -> placed(faces, next, turning));
    }

    Checked<Launcher> carriedTo(Frame to) {
        return Course.of(to, course.ends(), to, course.ends()).map(still -> placed(faces, still, wheel));
    }

    Launcher carriedMovingAsItWasTo(Frame to) {
        return placed(faces, course.movingAsItWasAt(to), wheel);
    }

    boolean standsAs(Launcher other) {
        return !course.moving()
                && !other.course.moving()
                && course.endsAt().equals(other.course.endsAt())
                && wheel.radiansPerSecond() == other.wheel.radiansPerSecond();
    }

    boolean holds(Vec3 centre, Length radius) {
        return !squeezing(centre, radius).isEmpty()
                || !near(centre, radius, Shot.TOUCHING_IN).isEmpty();
    }

    Checked<Bodies.Motion> carried(Bodies.Seen ball, Launcher to) {
        Vec3 within = unturned(ball.at().minus(where.origin()));
        Vec3 moving = unturned(ball.velocity().minus(frameVelocityAt(ball.at())));
        Vec3 spinning = unturned(ball.spin().minus(frameSpin));
        Vec3 at = to.world(within);
        return Bodies.Motion.of(
                at,
                to.turned(moving).plus(to.frameVelocityAt(at)),
                to.turned(spinning).plus(to.frameSpin));
    }

    private static double inches(double millimetres) {
        return millimetres / MILLIMETRES_PER_INCH;
    }

    private static double radians(double degrees) {
        return degrees * StrictMath.PI / 180;
    }

    private static Vec3 hub() {
        return new Vec3(-inches(AXLE_BEHIND_CENTRE_MM), 0, inches(AXLE_UP_MM));
    }

    private static Vec3 axle() {
        return new Vec3(0, 1, 0);
    }

    private Vec3 local(Vec3 world) {
        return unturned(world.minus(where.origin()));
    }

    private Vec3 unturned(Vec3 off) {
        return new Vec3(cos * off.x() + sin * off.y(), -sin * off.x() + cos * off.y(), off.z());
    }

    private Vec3 turned(Vec3 local) {
        return new Vec3(cos * local.x() - sin * local.y(), sin * local.x() + cos * local.y(), local.z());
    }

    private Vec3 world(Vec3 local) {
        return where.origin().plus(turned(local));
    }

    private Vec3 frameVelocityAt(Vec3 point) {
        return course.velocity().plus(frameSpin.cross(point.minus(where.origin())));
    }

    Placed where() {
        return where;
    }

    Vec3 entry(Length radius) {
        return world(hub().plus(Way.at(radians(EXTENSION_FROM_DEGREES), inches(GROOVE_BOTTOM_MM) + radius.inches())
                .local()));
    }

    Vec3 feed(Length radius) {
        Vec3 up = turned(
                Way.at(radians(EXTENSION_FROM_DEGREES) + StrictMath.PI / 2, 1).local());
        return frameVelocityAt(entry(radius)).plus(up.times(FEED_M_PER_S / Length.METRES_PER_INCH));
    }

    @Override
    public List<String> ids() {
        List<String> ids = new ArrayList<>();
        for (WheelRing ring : faces.wheel) {
            ids.add(ring.id());
        }
        for (HoodRing ring : faces.hood) {
            ids.add(ring.id());
        }
        ids.add(STOP);
        return ids;
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        Vec3 local = local(centre);
        Vec3 hub = hub();
        double out = StrictMath.hypot(local.x() - hub.x(), local.z() - hub.z());
        if (!(out >= inches(INNERMOST_MM) - by
                && out <= inches(OUTERMOST_MM) + by
                && StrictMath.abs(local.y()) <= inches(WIDEST_MM) + by)) {
            return false;
        }
        for (WheelRing ring : faces.wheel) {
            if (ring.arc().nearestTo(local).distance() <= by) {
                return true;
            }
        }
        for (HoodRing ring : faces.hood) {
            if (ring.nearestTo(local).distance() <= by) {
                return true;
            }
        }
        return faces.stop.reaches(local, by) && faces.stop.near(local, 0).gap() <= by;
    }

    @Override
    public double sweep() {
        return course.sweep();
    }

    @Override
    public boolean moving() {
        return course.moving() || wheel.radiansPerSecond() != 0;
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        Vec3 local = local(centre);
        double r = radius.inches();
        List<Surroundings.Touch> touches = new ArrayList<>();
        if (faces.stop.reaches(local, r + within)) {
            Colliders.Near near = faces.stop.near(local, r);
            if (near.gap() <= within) {
                Vec3 normal = turned(near.normal());
                Vec3 contact = centre.along(normal, -(r + near.gap()));
                Surroundings.Touch.of(STOP, Surface.FRAME, normal, near.gap(), frameVelocityAt(contact), frameSpin)
                        .fold(touch -> touches.add(touch), rule -> false);
            }
        }
        return touches;
    }

    @Override
    public boolean squeezes() {
        return true;
    }

    @Override
    public List<Surroundings.Overlap> squeezing(Vec3 centre, Length radius) {
        Vec3 local = local(centre);
        List<Surroundings.Overlap> overlaps = new ArrayList<>();
        for (WheelRing ring : faces.wheel) {
            overlap(overlaps, ring.id(), faces.wheelSurface, ring.arc().nearestTo(local), radius, wheelSupport);
        }
        for (HoodRing ring : faces.hood) {
            overlap(overlaps, ring.id(), faces.hoodSurface, ring.nearestTo(local), radius, hoodSupport);
        }
        return overlaps;
    }

    private void overlap(
            List<Surroundings.Overlap> overlaps,
            String id,
            Compliance surface,
            Arc.Nearest nearest,
            Length radius,
            Checked<Surroundings.Support> support) {
        double depth = radius.inches() - nearest.distance();
        if (depth > 0) {
            support.then(moving -> Surroundings.Overlap.of(id, surface, turned(nearest.normal()), depth, moving))
                    .fold(made -> overlaps.add(made), rule -> false);
        }
    }

    @Override
    public Launcher at(GridTime time) {
        return new Launcher(faces, course, wheel, time);
    }

    @Override
    public String toString() {
        return "the launcher on a course " + course + ", its wheel at " + wheel + ", placed at " + where;
    }
}
