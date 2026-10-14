package org.firstinspires.ftc.teamcode.simcore;

final class Launcher {
    static final String CAD_SOURCE =
            "Reginald's CAD, the 13 October snapshot's exact faces, with the hood and the lead-in"
                    + " as the team allowed them changed: doc/plans/runtime-flight.md, stage 4, What the CAD says and What we"
                    + " change";
    static final double MILLIMETRES_PER_INCH = 25.4;
    static final double AXLE_BEHIND_CENTRE_MM = 217 - 204;
    static final double AXLE_UP_MM = 180;
    static final double WHEEL_TORUS_MM = 75.2886;
    static final double WHEEL_TUBE_MM = 35.56;
    static final double HOOD_TUBE_MM = 36;
    static final String POLLEN_SOURCE = "a flight's pollen is 1.39 in in radius, as the field's pieces are: the"
            + " simulated 70.61 mm pollen the plan's hood squeezes by 2.0 mm";
    static final double POLLEN_RADIUS_IN = 1.39;
    static final double LEAD_IN_SQUEEZE_MM = 0.25;
    static final double GROOVE_BOTTOM_MM = WHEEL_TORUS_MM - WHEEL_TUBE_MM;
    static final double LEAD_IN_LOWER_END_MM =
            GROOVE_BOTTOM_MM + 2 * POLLEN_RADIUS_IN * MILLIMETRES_PER_INCH - LEAD_IN_SQUEEZE_MM;
    static final double OUTERMOST_MM = StrictMath.hypot(LEAD_IN_LOWER_END_MM, POLLEN_RADIUS_IN * MILLIMETRES_PER_INCH);
    static final double WIDEST_MM = HOOD_TUBE_MM;
    static final double FARTHEST_IN =
            StrictMath.hypot(AXLE_BEHIND_CENTRE_MM + OUTERMOST_MM, WIDEST_MM) / MILLIMETRES_PER_INCH;

    record Placed(Vec3 origin, double yaw) {}

    static final class Frame {
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

        static Checked<Frame> of(double x, double y, double yaw) {
            if (!(StrictMath.abs(x) <= MOST_FROM_CENTRE_IN && StrictMath.abs(y) <= MOST_FROM_CENTRE_IN)) {
                return Checked.rejected("a launcher's frame is within " + MOST_FROM_CENTRE_IN
                        + " in of the field's centre either way, not at (" + x + ", " + y + ")");
            }
            if (!Double.isFinite(yaw)) {
                return Checked.rejected("a launcher's frame faces a finite yaw, not " + yaw);
            }
            return Checked.ok(new Frame(x, y, StrictMath.IEEEremainder(yaw, Flywheel.WHOLE_TURN)));
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
        private final double seconds;
        private final double acrossX;
        private final double acrossY;
        private final double turned;

        private Course(
                Frame from, GridTime began, Frame to, double seconds, double acrossX, double acrossY, double turned) {
            this.from = from;
            this.began = began;
            this.to = to;
            this.seconds = seconds;
            this.acrossX = acrossX;
            this.acrossY = acrossY;
            this.turned = turned;
        }

        static Checked<Course> of(Frame from, GridTime began, Frame to, GridTime ends) {
            double seconds = between(began, ends);
            if (!(seconds >= 0)) {
                return Checked.rejected(
                        "a launcher's course ends no earlier than it begins, not at " + ends + " from " + began);
            }
            if (!(seconds > 0)) {
                return Checked.ok(new Course(from, began, to, 0, 0, 0, 0));
            }
            Course course = new Course(
                    from,
                    began,
                    to,
                    seconds,
                    to.x - from.x,
                    to.y - from.y,
                    StrictMath.IEEEremainder(to.yaw - from.yaw, Flywheel.WHOLE_TURN));
            Vec3 velocity = course.velocity();
            double sweep = StrictMath.sqrt(velocity.dot(velocity)) + StrictMath.abs(course.turning()) * FARTHEST_IN;
            if (!(sweep <= Flight.MOST_SPEED_IN_PER_S)) {
                return Checked.rejected("a launcher's frame sweeps its faces at most " + Flight.MOST_SPEED_IN_PER_S
                        + " in/s, as fast as a flight takes a ball, not " + sweep + " in/s, going at " + velocity
                        + " in/s and turning at " + course.turning() + " rad/s");
            }
            return Checked.ok(course);
        }

        private static double between(GridTime earlier, GridTime later) {
            return later.seconds() - earlier.seconds();
        }

        Vec3 velocity() {
            if (!(seconds > 0)) {
                return Vec3.zero();
            }
            return new Vec3(acrossX / seconds, acrossY / seconds, 0);
        }

        double turning() {
            if (!(seconds > 0)) {
                return 0;
            }
            return turned / seconds;
        }

        double sweep() {
            Vec3 velocity = velocity();
            return StrictMath.sqrt(velocity.dot(velocity)) + StrictMath.abs(turning()) * FARTHEST_IN;
        }

        boolean moving() {
            return acrossX != 0 || acrossY != 0 || turned != 0;
        }

        Placed at(GridTime time) {
            if (!(seconds > 0)) {
                return new Placed(new Vec3(to.x, to.y, 0), to.yaw);
            }
            double part = between(began, time) / seconds;
            if (part < 0.5) {
                return new Placed(
                        new Vec3(from.x + part * acrossX, from.y + part * acrossY, 0), from.yaw + part * turned);
            }
            double left = part - 1;
            return new Placed(new Vec3(to.x + left * acrossX, to.y + left * acrossY, 0), to.yaw + left * turned);
        }

        @Override
        public String toString() {
            return "from " + from + " at " + began + " to " + to + " " + seconds + " s later";
        }
    }

    private Launcher() {}
}
