package org.firstinspires.ftc.teamcode.simcore;

public final class WheelPair {
    public static final double SECONDS_PER_MINUTE = 60;

    private final Vec3 direction;
    private final Vec3 towardWheelOne;
    private final Launched launched;

    private WheelPair(Vec3 direction, Vec3 towardWheelOne, Launched launched) {
        this.direction = direction;
        this.towardWheelOne = towardWheelOne;
        this.launched = launched;
    }

    public static Checked<WheelPair> of(
            Ball pollen,
            Vec3 exit,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees,
            Flywheel one,
            Flywheel two,
            double speedEfficiency,
            double spinEfficiency,
            double compressionInches,
            Inertia inertia,
            Vec2 robotVelocity) {
        if (!exit.isFinite()) {
            return Checked.rejected("a launch leaves from a finite exit point, not " + exit);
        }
        if (!(Double.isFinite(yawDegrees) && Double.isFinite(pitchDegrees) && Double.isFinite(rollDegrees))) {
            return Checked.rejected("a launcher's yaw, pitch and roll are finite numbers of degrees, not yaw "
                    + yawDegrees + ", pitch " + pitchDegrees + ", roll " + rollDegrees);
        }
        if (!(speedEfficiency > 0 && speedEfficiency <= 1)) {
            return Checked.rejected(
                    "a launcher's speed efficiency is more than 0 and at most 1, not " + speedEfficiency);
        }
        if (!(spinEfficiency > 0 && spinEfficiency <= 1)) {
            return Checked.rejected("a launcher's spin efficiency is more than 0 and at most 1, not " + spinEfficiency);
        }
        if (!(compressionInches >= 0 && Double.isFinite(compressionInches))) {
            return Checked.rejected("the wheels press into the pollen by a finite number of inches, 0 or more, not "
                    + compressionInches);
        }
        double diameter = pollen.radius() * 2;
        if (!(compressionInches < diameter)) {
            return Checked.rejected("the wheels press into the pollen by less than its diameter, " + diameter
                    + " in, not " + compressionInches + " in");
        }
        if (!(Double.isFinite(robotVelocity.x()) && Double.isFinite(robotVelocity.y()))) {
            return Checked.rejected("the robot moves at a finite velocity, not " + robotVelocity);
        }
        Frame frame = Frame.of(yawDegrees, pitchDegrees, rollDegrees);
        double contactRadius = (diameter - compressionInches) / 2;
        double surfaceOne = one.surfaceSpeed(), surfaceTwo = two.surfaceSpeed();
        Thrown thrown = inertia instanceof Inertia.Slows slows
                ? Thrown.byWheelsThatSlow(pollen, contactRadius, one, two, slows)
                : Thrown.byWheelsThatKeepTheirSpeed(surfaceOne, surfaceTwo);
        Vec3 velocity = frame.direction()
                .times(thrown.speed() * speedEfficiency)
                .plus(new Vec3(robotVelocity.x(), robotVelocity.y(), 0));
        Vec3 spin = frame.towardWheelOne()
                .cross(frame.direction())
                .times(thrown.rimSpeed() / contactRadius * spinEfficiency);
        Checked<Launched> launched = Launched.of(
                exit,
                velocity,
                spin,
                surfaceOne,
                surfaceTwo,
                thrown.speed() + thrown.rimSpeed(),
                thrown.speed() - thrown.rimSpeed());
        if (!(launched instanceof Checked.Ok<Launched> ok)) {
            return Checked.rejected("a launch leaves at a finite speed and spin, and " + one + " and " + two
                    + " throwing " + pollen + " pressed in by " + compressionInches + " in would not");
        }
        return Checked.ok(new WheelPair(frame.direction(), frame.towardWheelOne(), ok.value()));
    }

    public Launched launch() {
        return launched;
    }

    public Vec3 direction() {
        return direction;
    }

    public Vec3 towardWheelOne() {
        return towardWheelOne;
    }

    private record Frame(Vec3 direction, Vec3 towardWheelOne) {
        static Frame of(double yawDegrees, double pitchDegrees, double rollDegrees) {
            double yaw = StrictMath.toRadians(yawDegrees);
            double pitch = StrictMath.toRadians(pitchDegrees);
            double roll = StrictMath.toRadians(rollDegrees);
            double cosYaw = StrictMath.cos(yaw), sinYaw = StrictMath.sin(yaw);
            double cosPitch = StrictMath.cos(pitch), sinPitch = StrictMath.sin(pitch);
            Vec3 direction = new Vec3(cosPitch * cosYaw, cosPitch * sinYaw, sinPitch);
            Vec3 left = new Vec3(-sinYaw, cosYaw, 0);
            Vec3 top = direction.cross(left);
            Vec3 towardWheelOne = top.times(StrictMath.cos(roll)).plus(left.times(StrictMath.sin(roll)));
            return new Frame(direction, towardWheelOne);
        }
    }

    private record Thrown(double speed, double rimSpeed) {
        static Thrown byWheelsThatKeepTheirSpeed(double surfaceOne, double surfaceTwo) {
            return new Thrown((surfaceOne + surfaceTwo) / 2, (surfaceOne - surfaceTwo) / 2);
        }

        static Thrown byWheelsThatSlow(
                Ball pollen, double contactRadius, Flywheel one, Flywheel two, Inertia.Slows slows) {
            double ball = pollen.kilograms();
            double radii = pollen.radius() / contactRadius;
            double ballTurning = pollen.inertiaFactor() * ball * radii * radii;
            double wheelOne = slows.rimMassOf(one), wheelTwo = slows.rimMassOf(two);
            double surfaceOne = one.surfaceSpeed(), surfaceTwo = two.surfaceSpeed();
            double both = wheelOne + wheelTwo, between = wheelOne - wheelTwo;
            double forwards = wheelOne * surfaceOne + wheelTwo * surfaceTwo;
            double turning = wheelOne * surfaceOne - wheelTwo * surfaceTwo;
            double determinant = ball * ballTurning + (ball + ballTurning) * both + 4 * wheelOne * wheelTwo;
            double speed = (forwards * (ballTurning + both) - between * turning) / determinant;
            double rimSpeed = ((ball + both) * turning - between * forwards) / determinant;
            return new Thrown(speed, rimSpeed);
        }
    }

    public static final class Flywheel {
        private final double diameterInches;
        private final double rpm;
        private final double surfaceSpeed;

        private Flywheel(double diameterInches, double rpm, double surfaceSpeed) {
            this.diameterInches = diameterInches;
            this.rpm = rpm;
            this.surfaceSpeed = surfaceSpeed;
        }

        public static Checked<Flywheel> of(double diameterInches, double rpm) {
            if (!(diameterInches > 0 && Double.isFinite(diameterInches))) {
                return Checked.rejected(
                        "a flywheel's diameter is a positive, finite number of inches, not " + diameterInches);
            }
            if (!(rpm >= 0 && Double.isFinite(rpm))) {
                return Checked.rejected("a flywheel turns forwards at a finite rpm, 0 or more, not " + rpm + " rpm");
            }
            double surfaceSpeed = StrictMath.PI * diameterInches * rpm / SECONDS_PER_MINUTE;
            if (!Double.isFinite(surfaceSpeed)) {
                return Checked.rejected("a flywheel's rim moves at a finite speed, and one " + diameterInches
                        + " in across at " + rpm + " rpm would not");
            }
            return Checked.ok(new Flywheel(diameterInches, rpm, surfaceSpeed));
        }

        public double diameterInches() {
            return diameterInches;
        }

        public double rpm() {
            return rpm;
        }

        public double surfaceSpeed() {
            return surfaceSpeed;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Flywheel that
                    && Double.compare(diameterInches, that.diameterInches) == 0
                    && Double.compare(rpm, that.rpm) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(diameterInches) * 31 + Double.hashCode(rpm);
        }

        @Override
        public String toString() {
            return "a flywheel " + diameterInches + " in across at " + rpm + " rpm";
        }
    }

    public sealed interface Inertia permits Inertia.KeepSpeed, Inertia.Slows {
        record KeepSpeed() implements Inertia {}

        final class Slows implements Inertia {
            public static final double KILOGRAMS_PER_POUND = 0.45359237;

            private final double kilogramSquareMetres;

            private Slows(double kilogramSquareMetres) {
                this.kilogramSquareMetres = kilogramSquareMetres;
            }

            public static Checked<Slows> of(double kilogramSquareMetres) {
                if (!(kilogramSquareMetres > 0 && Double.isFinite(kilogramSquareMetres))) {
                    return Checked.rejected("a flywheel that slows has a positive, finite moment of inertia in kg"
                            + " m^2, not " + kilogramSquareMetres);
                }
                return Checked.ok(new Slows(kilogramSquareMetres));
            }

            public static double fromPoundSquareInches(double poundSquareInches) {
                return poundSquareInches * KILOGRAMS_PER_POUND * Length.METRES_PER_INCH * Length.METRES_PER_INCH;
            }

            public double kilogramSquareMetres() {
                return kilogramSquareMetres;
            }

            double rimMassOf(Flywheel wheel) {
                double radiusMetres = wheel.diameterInches() / 2 * Length.METRES_PER_INCH;
                return kilogramSquareMetres / (radiusMetres * radiusMetres);
            }

            @Override
            public boolean equals(Object other) {
                return other instanceof Slows that
                        && Double.compare(kilogramSquareMetres, that.kilogramSquareMetres) == 0;
            }

            @Override
            public int hashCode() {
                return Double.hashCode(kilogramSquareMetres);
            }

            @Override
            public String toString() {
                return "flywheels that slow, each " + kilogramSquareMetres + " kg m^2";
            }
        }
    }

    public static final class Launched {
        private final Vec3 at;
        private final Vec3 velocity;
        private final Vec3 spin;
        private final double surfaceOne;
        private final double surfaceTwo;
        private final double surfaceOneAfter;
        private final double surfaceTwoAfter;

        private Launched(
                Vec3 at,
                Vec3 velocity,
                Vec3 spin,
                double surfaceOne,
                double surfaceTwo,
                double surfaceOneAfter,
                double surfaceTwoAfter) {
            this.at = at;
            this.velocity = velocity;
            this.spin = spin;
            this.surfaceOne = surfaceOne;
            this.surfaceTwo = surfaceTwo;
            this.surfaceOneAfter = surfaceOneAfter;
            this.surfaceTwoAfter = surfaceTwoAfter;
        }

        public static Checked<Launched> of(
                Vec3 at,
                Vec3 velocity,
                Vec3 spin,
                double surfaceOne,
                double surfaceTwo,
                double surfaceOneAfter,
                double surfaceTwoAfter) {
            Launched launched =
                    new Launched(at, velocity, spin, surfaceOne, surfaceTwo, surfaceOneAfter, surfaceTwoAfter);
            if (!(at.isFinite()
                    && velocity.isFinite()
                    && spin.isFinite()
                    && Double.isFinite(surfaceOne)
                    && Double.isFinite(surfaceTwo)
                    && Double.isFinite(surfaceOneAfter)
                    && Double.isFinite(surfaceTwoAfter))) {
                return Checked.rejected("a pollen leaves the wheels from a finite place at a finite speed and spin,"
                        + " not " + launched);
            }
            return Checked.ok(launched);
        }

        public Vec3 at() {
            return at;
        }

        public Vec3 velocity() {
            return velocity;
        }

        public Vec3 spin() {
            return spin;
        }

        public double surfaceOne() {
            return surfaceOne;
        }

        public double surfaceTwo() {
            return surfaceTwo;
        }

        public double surfaceOneAfter() {
            return surfaceOneAfter;
        }

        public double surfaceTwoAfter() {
            return surfaceTwoAfter;
        }

        public SpinKind kind() {
            return SpinKind.of(velocity, spin);
        }

        public double spinRpm() {
            return StrictMath.sqrt(spin.dot(spin)) * SECONDS_PER_MINUTE / (2 * StrictMath.PI);
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Launched that
                    && at.equals(that.at)
                    && velocity.equals(that.velocity)
                    && spin.equals(that.spin)
                    && Double.compare(surfaceOne, that.surfaceOne) == 0
                    && Double.compare(surfaceTwo, that.surfaceTwo) == 0
                    && Double.compare(surfaceOneAfter, that.surfaceOneAfter) == 0
                    && Double.compare(surfaceTwoAfter, that.surfaceTwoAfter) == 0;
        }

        @Override
        public int hashCode() {
            return (at.hashCode() * 31 + velocity.hashCode()) * 31 + spin.hashCode();
        }

        @Override
        public String toString() {
            return "launched from " + at + " at " + velocity + " spinning " + spin + ", the wheels' surfaces "
                    + surfaceOne + " and " + surfaceTwo + " in/s, after " + surfaceOneAfter + " and "
                    + surfaceTwoAfter;
        }
    }

    public enum SpinKind {
        NONE,
        BACKSPIN,
        TOPSPIN,
        SIDESPIN,
        RIFLE;

        public static SpinKind of(Vec3 velocity, Vec3 spin) {
            if (spin.x() == 0 && spin.y() == 0 && spin.z() == 0) {
                return NONE;
            }
            Vec3 going = scaledToAboutOne(velocity);
            Vec3 turning = scaledToAboutOne(spin);
            double along = turning.dot(going);
            double speedSquared = going.dot(going);
            if (2 * along * along > turning.dot(turning) * speedSquared) {
                return RIFLE;
            }
            Vec3 magnus = turning.cross(going);
            double rising = magnus.z();
            double level = going.x() * going.x() + going.y() * going.y();
            boolean steep = 2 * speedSquared * rising * rising > magnus.dot(magnus) * level;
            if (steep && rising > 0) {
                return BACKSPIN;
            }
            if (steep && rising < 0) {
                return TOPSPIN;
            }
            return SIDESPIN;
        }

        private static Vec3 scaledToAboutOne(Vec3 v) {
            double largest =
                    StrictMath.max(StrictMath.abs(v.x()), StrictMath.max(StrictMath.abs(v.y()), StrictMath.abs(v.z())));
            return largest > 0 && Double.isFinite(largest)
                    ? new Vec3(v.x() / largest, v.y() / largest, v.z() / largest)
                    : v;
        }
    }
}
