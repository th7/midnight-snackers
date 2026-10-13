package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class WheelPairTest {
    private static final double DELTA = 1e-9;
    private static final double POLLEN_DIAMETER = 2.8;
    private static final double WHEEL_DIAMETER = 3.78;
    private static final Vec3 EXIT = new Vec3(10, -20, 14);
    private static final Vec2 STILL = new Vec2(0, 0);

    private static final class Asked {
        double pollenDiameter = POLLEN_DIAMETER;
        Vec3 exit = EXIT;
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        double diameterOne = WHEEL_DIAMETER;
        double rpmOne = 1200;
        double diameterTwo = WHEEL_DIAMETER;
        double rpmTwo = 900;
        double speedEfficiency = 1;
        double spinEfficiency = 1;
        double compression = 0;
        WheelPair.Inertia inertia = new WheelPair.Inertia.KeepSpeed();
        Vec2 robot = STILL;

        Ball pollen() {
            return Valid.value(Ball.of(pollenDiameter, 24.9, 0.634));
        }

        Checked<WheelPair> pair() {
            return WheelPair.Flywheel.of(diameterOne, rpmOne)
                    .then(one -> WheelPair.Flywheel.of(diameterTwo, rpmTwo)
                            .then(two -> WheelPair.of(
                                    pollen(),
                                    exit,
                                    yaw,
                                    pitch,
                                    roll,
                                    one,
                                    two,
                                    speedEfficiency,
                                    spinEfficiency,
                                    compression,
                                    inertia,
                                    robot)));
        }

        WheelPair.Launched launched() {
            return Valid.value(pair()).launch();
        }
    }

    private static double surface(double diameter, double rpm) {
        return Math.PI * diameter * rpm / 60;
    }

    private static double length(Vec3 v) {
        return Math.sqrt(v.dot(v));
    }

    private static void assertVec(Vec3 expected, Vec3 actual) {
        assertEquals("x of " + actual, expected.x(), actual.x(), DELTA);
        assertEquals("y of " + actual, expected.y(), actual.y(), DELTA);
        assertEquals("z of " + actual, expected.z(), actual.z(), DELTA);
    }

    private static void assertExactly(Vec3 expected, Vec3 actual) {
        assertEquals("x of " + actual, expected.x(), actual.x(), 0);
        assertEquals("y of " + actual, expected.y(), actual.y(), 0);
        assertEquals("z of " + actual, expected.z(), actual.z(), 0);
    }

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static WheelPair.Inertia slows(double kilogramSquareMetres) {
        return Valid.value(WheelPair.Inertia.Slows.of(kilogramSquareMetres));
    }

    private static WheelPair.Launched flying(Vec3 velocity, Vec3 spin) {
        return Valid.value(WheelPair.Launched.of(EXIT, velocity, spin, 0, 0, 0, 0));
    }

    private static Vec3 spinWhoseMagnusRises(double degrees) {
        double rises = Math.toRadians(degrees);
        return new Vec3(0, -Math.sin(rises), Math.cos(rises));
    }

    private static Vec3 spinThisFarFromTheWayItGoes(double degrees) {
        double off = Math.toRadians(degrees);
        return new Vec3(Math.cos(off), 0, Math.sin(off));
    }

    @Test
    public void aFlywheelsRimMovesAtPiTimesItsDiameterEachTurn() {
        WheelPair.Flywheel wheel = Valid.value(WheelPair.Flywheel.of(WHEEL_DIAMETER, 1200));

        assertEquals(Math.PI * WHEEL_DIAMETER * 20, wheel.surfaceSpeed(), DELTA);
        assertEquals(WHEEL_DIAMETER, wheel.diameterInches(), 0);
        assertEquals(1200, wheel.rpm(), 0);
        assertEquals(Valid.value(WheelPair.Flywheel.of(WHEEL_DIAMETER, 1200)), wheel);
        assertEquals(0, Valid.value(WheelPair.Flywheel.of(WHEEL_DIAMETER, 0)).surfaceSpeed(), 0);
    }

    @Test
    public void equalWheelsThrowWithoutSpinAtTheirSurfaceSpeed() {
        Asked asked = new Asked();
        asked.rpmOne = 1000;
        asked.rpmTwo = 1000;

        WheelPair.Launched launched = asked.launched();

        assertExactly(Vec3.zero(), launched.spin());
        assertVec(new Vec3(surface(WHEEL_DIAMETER, 1000), 0, 0), launched.velocity());
        assertEquals(WheelPair.SpinKind.NONE, launched.kind());
        assertEquals(0, launched.spinRpm(), 0);
    }

    @Test
    public void itLeavesFromTheExit() {
        assertExactly(EXIT, new Asked().launched().at());
    }

    @Test
    public void wheelOneOnTopTurningFasterGivesTopspin() {
        Asked asked = new Asked();
        asked.rpmOne = 1200;
        asked.rpmTwo = 900;

        WheelPair.Launched launched = asked.launched();

        double one = surface(WHEEL_DIAMETER, 1200), two = surface(WHEEL_DIAMETER, 900);
        assertVec(new Vec3((one + two) / 2, 0, 0), launched.velocity());
        assertVec(new Vec3(0, (one - two) / POLLEN_DIAMETER, 0), launched.spin());
        assertEquals(WheelPair.SpinKind.TOPSPIN, launched.kind());
    }

    @Test
    public void wheelTwoUnderneathTurningFasterGivesBackspin() {
        Asked asked = new Asked();
        asked.rpmOne = 900;
        asked.rpmTwo = 1200;

        WheelPair.Launched launched = asked.launched();

        double one = surface(WHEEL_DIAMETER, 900), two = surface(WHEEL_DIAMETER, 1200);
        assertVec(new Vec3(0, (one - two) / POLLEN_DIAMETER, 0), launched.spin());
        assertTrue("spin about -y: " + launched.spin(), launched.spin().y() < 0);
        assertEquals(WheelPair.SpinKind.BACKSPIN, launched.kind());
    }

    @Test
    public void eachWheelsSurfaceSpeedIsKeptWhenTheWheelsKeepTheirSpeed() {
        Asked asked = new Asked();
        asked.diameterOne = 4;
        asked.rpmOne = 1500;
        asked.diameterTwo = 3;
        asked.rpmTwo = 700;

        WheelPair.Launched launched = asked.launched();

        assertEquals(surface(4, 1500), launched.surfaceOne(), DELTA);
        assertEquals(surface(3, 700), launched.surfaceTwo(), DELTA);
        assertEquals(launched.surfaceOne(), launched.surfaceOneAfter(), DELTA);
        assertEquals(launched.surfaceTwo(), launched.surfaceTwoAfter(), DELTA);
    }

    @Test
    public void swappingTheWheelsSpeedsFlipsTheSpinExactly() {
        Asked asked = new Asked();
        asked.yaw = 37;
        asked.pitch = 23;
        asked.roll = 11;
        asked.rpmOne = 1730;
        asked.rpmTwo = 640;
        WheelPair.Launched before = asked.launched();
        asked.rpmOne = 640;
        asked.rpmTwo = 1730;

        WheelPair.Launched swapped = asked.launched();

        assertExactly(before.spin().negated(), swapped.spin());
        assertExactly(before.velocity(), swapped.velocity());
    }

    @Test
    public void rollingTheWheelsAQuarterTurnStandsTheSpinAxisUpright() {
        Asked asked = new Asked();
        asked.roll = 90;

        WheelPair.Launched launched = asked.launched();

        double spin = (surface(WHEEL_DIAMETER, 1200) - surface(WHEEL_DIAMETER, 900)) / POLLEN_DIAMETER;
        assertVec(new Vec3(0, 0, -spin), launched.spin());
        assertEquals(WheelPair.SpinKind.SIDESPIN, launched.kind());
    }

    @Test
    public void rollingTheWheelsAHalfTurnReversesTheSpin() {
        Asked asked = new Asked();
        asked.yaw = 25;
        asked.pitch = 40;
        Vec3 upright = asked.launched().spin();
        asked.roll = 180;

        assertVec(upright.negated(), asked.launched().spin());
    }

    @Test
    public void wheelOneIsOnTopAtNoRollAndOnTheLeftAtAQuarterTurn() {
        Asked asked = new Asked();
        asked.yaw = 90;

        assertVec(new Vec3(0, 0, 1), Valid.value(asked.pair()).towardWheelOne());
        asked.roll = 90;
        assertVec(new Vec3(-1, 0, 0), Valid.value(asked.pair()).towardWheelOne());
    }

    @Test
    public void theDirectionIsTheYawAroundAndThePitchUp() {
        Asked asked = new Asked();
        asked.yaw = 30;
        asked.pitch = 40;

        double yaw = Math.toRadians(30), pitch = Math.toRadians(40);
        assertVec(
                new Vec3(Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)),
                Valid.value(asked.pair()).direction());
    }

    @Test
    public void aYawOfNinetySendsItAlongPlusY() {
        Asked asked = new Asked();
        asked.yaw = 90;

        assertVec(
                new Vec3(0, (surface(WHEEL_DIAMETER, 1200) + surface(WHEEL_DIAMETER, 900)) / 2, 0),
                asked.launched().velocity());
    }

    @Test
    public void aPitchOfNinetySendsItStraightUp() {
        Asked asked = new Asked();
        asked.pitch = 90;

        WheelPair.Launched launched = asked.launched();

        double speed = (surface(WHEEL_DIAMETER, 1200) + surface(WHEEL_DIAMETER, 900)) / 2;
        double spin = (surface(WHEEL_DIAMETER, 1200) - surface(WHEEL_DIAMETER, 900)) / POLLEN_DIAMETER;
        assertVec(new Vec3(0, 0, speed), launched.velocity());
        assertTrue(launched.velocity().isFinite() && launched.spin().isFinite());
        assertEquals(spin, length(launched.spin()), DELTA);
        assertEquals(0, launched.spin().dot(launched.velocity()), DELTA);
    }

    @Test
    public void theDefaultLaunchIsALobWithTwoHundredRpmOfBackspin() {
        Asked asked = new Asked();
        asked.yaw = 180;
        asked.pitch = 70;
        asked.rpmOne = 900;
        asked.rpmTwo = 1200;

        WheelPair.Launched launched = asked.launched();

        assertEquals(surface(WHEEL_DIAMETER, 1050), length(launched.velocity()), DELTA);
        assertEquals(WHEEL_DIAMETER * 300 / (2 * POLLEN_DIAMETER), launched.spinRpm(), DELTA);
        assertEquals(202.5, launched.spinRpm(), DELTA);
        assertEquals(WheelPair.SpinKind.BACKSPIN, launched.kind());
    }

    @Test
    public void efficienciesScaleTheSpeedAndTheSpinEachOnItsOwn() {
        Asked asked = new Asked();
        asked.yaw = 60;
        asked.pitch = 30;
        asked.roll = 20;
        WheelPair.Launched ideal = asked.launched();
        asked.speedEfficiency = 0.8;
        WheelPair.Launched slower = asked.launched();
        asked.speedEfficiency = 1;
        asked.spinEfficiency = 0.5;
        WheelPair.Launched lessSpun = asked.launched();

        assertVec(ideal.velocity().times(0.8), slower.velocity());
        assertVec(ideal.spin(), slower.spin());
        assertVec(ideal.velocity(), lessSpun.velocity());
        assertVec(ideal.spin().times(0.5), lessSpun.spin());
    }

    @Test
    public void squeezingTheBallSpinsItFasterByItsDiameterOverWhatIsLeftAndLeavesItsSpeed() {
        Asked asked = new Asked();
        asked.yaw = -40;
        asked.pitch = 15;
        WheelPair.Launched loose = asked.launched();
        asked.compression = 0.4;

        WheelPair.Launched squeezed = asked.launched();

        assertVec(loose.spin().times(POLLEN_DIAMETER / (POLLEN_DIAMETER - 0.4)), squeezed.spin());
        assertVec(loose.velocity(), squeezed.velocity());
    }

    @Test
    public void wheelsThatSlowLoseWhatTheBallGains() {
        Asked asked = new Asked();
        asked.yaw = 150;
        asked.pitch = 55;
        asked.roll = 30;
        asked.diameterOne = 4;
        asked.rpmOne = 2400;
        asked.diameterTwo = 3;
        asked.rpmTwo = 1100;
        asked.compression = 0.3;
        double inertia = WheelPair.Inertia.Slows.fromPoundSquareInches(0.95);
        asked.inertia = slows(inertia);
        WheelPair pair = Valid.value(asked.pair());

        WheelPair.Launched launched = pair.launch();

        Ball pollen = asked.pollen();
        double contact = (POLLEN_DIAMETER - 0.3) / 2;
        Vec3 axis = pair.towardWheelOne().cross(pair.direction());
        double speed = launched.velocity().dot(pair.direction());
        double spin = launched.spin().dot(axis);
        assertEquals(speed + contact * spin, launched.surfaceOneAfter(), DELTA);
        assertEquals(speed - contact * spin, launched.surfaceTwoAfter(), DELTA);
        double wheelOne = inertia / Math.pow(2 * Length.METRES_PER_INCH, 2);
        double wheelTwo = inertia / Math.pow(1.5 * Length.METRES_PER_INCH, 2);
        double fromOne = wheelOne * (surface(4, 2400) - launched.surfaceOneAfter());
        double fromTwo = wheelTwo * (surface(3, 1100) - launched.surfaceTwoAfter());
        assertEquals(pollen.kilograms() * speed, fromOne + fromTwo, DELTA);
        double ballInertia = pollen.inertiaFactor() * pollen.kilograms() * pollen.radius() * pollen.radius();
        assertEquals(ballInertia * spin, (fromOne - fromTwo) * contact, DELTA);
    }

    @Test
    public void wheelsThatSlowThrowSlowerAndWithLessSpinThanWheelsThatKeepTheirSpeed() {
        Asked asked = new Asked();
        asked.yaw = 180;
        asked.pitch = 70;
        asked.rpmOne = 900;
        asked.rpmTwo = 1200;
        WheelPair.Launched ideal = asked.launched();
        asked.inertia = slows(WheelPair.Inertia.Slows.fromPoundSquareInches(0.95));

        WheelPair.Launched slowed = asked.launched();

        assertTrue(
                length(slowed.velocity()) + " < " + length(ideal.velocity()),
                length(slowed.velocity()) < length(ideal.velocity()));
        assertTrue(slowed.spinRpm() + " < " + ideal.spinRpm(), slowed.spinRpm() < ideal.spinRpm());
        assertTrue(slowed.surfaceOneAfter() < slowed.surfaceOne());
        assertTrue(slowed.surfaceTwoAfter() < slowed.surfaceTwo());
        assertEquals(ideal.surfaceOne(), slowed.surfaceOne(), 0);
        assertEquals(WheelPair.SpinKind.BACKSPIN, slowed.kind());
    }

    @Test
    public void wheelsTooHeavyToSlowThrowAsIfTheyKeptTheirSpeed() {
        Asked asked = new Asked();
        asked.yaw = 20;
        asked.pitch = 35;
        asked.roll = -60;
        asked.diameterOne = 4;
        asked.diameterTwo = 3;
        asked.compression = 0.5;
        WheelPair.Launched ideal = asked.launched();
        asked.inertia = slows(1e6);

        WheelPair.Launched heavy = asked.launched();

        assertEquals(0, length(heavy.velocity().minus(ideal.velocity())) / length(ideal.velocity()), 1e-6);
        assertEquals(0, length(heavy.spin().minus(ideal.spin())) / length(ideal.spin()), 1e-6);
        assertEquals(1, heavy.surfaceOneAfter() / ideal.surfaceOne(), 1e-6);
        assertEquals(1, heavy.surfaceTwoAfter() / ideal.surfaceTwo(), 1e-6);
    }

    @Test
    public void aPoundSquareInchIsAbout0point00029KilogramSquareMetres() {
        assertEquals(2.9264e-4, WheelPair.Inertia.Slows.fromPoundSquareInches(1), 1e-8);
        assertEquals(0.45359237 * 0.0254 * 0.0254, WheelPair.Inertia.Slows.fromPoundSquareInches(1), 1e-18);
        assertEquals(0.5, Valid.value(WheelPair.Inertia.Slows.of(0.5)).kilogramSquareMetres(), 0);
    }

    @Test
    public void theRobotsVelocityCarriesTheBallButDoesNotSpinIt() {
        Asked asked = new Asked();
        asked.yaw = 75;
        asked.pitch = 50;
        asked.roll = 15;
        WheelPair.Launched still = asked.launched();
        asked.robot = new Vec2(30, -20);

        WheelPair.Launched moving = asked.launched();

        assertVec(still.velocity().plus(new Vec3(30, -20, 0)), moving.velocity());
        assertExactly(still.spin(), moving.spin());
    }

    @Test
    public void aSpinAlongTheWayTheBallGoesIsRifleSpin() {
        Asked asked = new Asked();
        asked.rpmOne = 300;
        asked.rpmTwo = 100;
        asked.robot = new Vec2(0, 100);

        assertEquals(WheelPair.SpinKind.RIFLE, asked.launched().kind());
    }

    @Test
    public void spinIsBackspinWhenItLiftsTheBallMoreThanHalfwayUpright() {
        Vec3 forwards = new Vec3(5, 0, 0);

        assertEquals(
                WheelPair.SpinKind.BACKSPIN,
                flying(forwards, spinWhoseMagnusRises(90)).kind());
        assertEquals(
                WheelPair.SpinKind.BACKSPIN,
                flying(forwards, spinWhoseMagnusRises(46)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinWhoseMagnusRises(44)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinWhoseMagnusRises(0)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinWhoseMagnusRises(-44)).kind());
        assertEquals(
                WheelPair.SpinKind.TOPSPIN,
                flying(forwards, spinWhoseMagnusRises(-46)).kind());
        assertEquals(
                WheelPair.SpinKind.TOPSPIN,
                flying(forwards, spinWhoseMagnusRises(-90)).kind());
        assertEquals(
                WheelPair.SpinKind.BACKSPIN,
                flying(forwards, spinWhoseMagnusRises(134)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinWhoseMagnusRises(136)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinWhoseMagnusRises(180)).kind());
    }

    @Test
    public void backspinIsJudgedAcrossTheWayTheBallGoesHoweverSteeplyItClimbs() {
        double climb = Math.toRadians(80);
        Vec3 steeply = new Vec3(Math.cos(climb), 0, Math.sin(climb));
        Vec3 upright = new Vec3(0, 0, 7);

        assertEquals(
                WheelPair.SpinKind.BACKSPIN, flying(steeply, new Vec3(0, -3, 0)).kind());
        assertEquals(
                WheelPair.SpinKind.TOPSPIN, flying(steeply, new Vec3(0, 3, 0)).kind());
        assertEquals(
                WheelPair.SpinKind.BACKSPIN,
                flying(steeply.negated(), new Vec3(0, 3, 0)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN, flying(upright, new Vec3(0, -3, 0)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(upright.negated(), new Vec3(2, 0, 0)).kind());
    }

    @Test
    public void spinIsRifleSpinWhenMoreOfItIsAlongTheWayTheBallGoesThanAcross() {
        Vec3 forwards = new Vec3(5, 0, 0);

        assertEquals(
                WheelPair.SpinKind.RIFLE,
                flying(forwards, spinThisFarFromTheWayItGoes(0)).kind());
        assertEquals(
                WheelPair.SpinKind.RIFLE,
                flying(forwards, spinThisFarFromTheWayItGoes(44)).kind());
        assertEquals(
                WheelPair.SpinKind.SIDESPIN,
                flying(forwards, spinThisFarFromTheWayItGoes(46)).kind());
        assertEquals(
                WheelPair.SpinKind.RIFLE,
                flying(forwards, spinThisFarFromTheWayItGoes(180)).kind());
        assertEquals(
                WheelPair.SpinKind.RIFLE,
                flying(forwards, spinThisFarFromTheWayItGoes(136)).kind());
    }

    @Test
    public void onlyNoSpinAtAllIsNoSpin() {
        Vec3 forwards = new Vec3(5, 0, 0);

        assertEquals(WheelPair.SpinKind.NONE, flying(forwards, Vec3.zero()).kind());
        assertEquals(
                WheelPair.SpinKind.NONE,
                flying(forwards, new Vec3(-0.0, 0, -0.0)).kind());
        assertEquals(
                WheelPair.SpinKind.BACKSPIN,
                flying(forwards, new Vec3(0, -1e-300, 0)).kind());
    }

    @Test
    public void theKindOfSpinDoesNotDependOnHowFastTheBallGoesOrSpins() {
        Vec3 forwards = new Vec3(5, 0, 0);
        Vec3 spin = spinWhoseMagnusRises(50);
        Vec3 rifle = spinThisFarFromTheWayItGoes(40);

        for (double by : new double[] {1e-200, 1e-20, 1, 1e20, 1e200}) {
            assertEquals(
                    "at " + by,
                    WheelPair.SpinKind.BACKSPIN,
                    flying(forwards.times(by), spin).kind());
            assertEquals(
                    "at " + by,
                    WheelPair.SpinKind.BACKSPIN,
                    flying(forwards, spin.times(by)).kind());
            assertEquals(
                    "at " + by,
                    WheelPair.SpinKind.RIFLE,
                    flying(forwards.times(by), rifle.times(by)).kind());
        }
    }

    @Test
    public void spinRpmIsTheSpinsTurnsEachMinute() {
        assertEquals(60, flying(new Vec3(1, 0, 0), new Vec3(0, 0, 2 * Math.PI)).spinRpm(), DELTA);
        assertEquals(
                30,
                flying(new Vec3(1, 0, 0), new Vec3(0, -0.6 * Math.PI, 0.8 * Math.PI))
                        .spinRpm(),
                DELTA);
    }

    @Test
    public void aFlywheelHasASizeAndTurnsForwards() {
        assertRejected("rpm", WheelPair.Flywheel.of(WHEEL_DIAMETER, -1));
        assertRejected("rpm", WheelPair.Flywheel.of(WHEEL_DIAMETER, Double.NaN));
        assertRejected("rpm", WheelPair.Flywheel.of(WHEEL_DIAMETER, Double.POSITIVE_INFINITY));
        assertRejected("diameter", WheelPair.Flywheel.of(0, 1000));
        assertRejected("diameter", WheelPair.Flywheel.of(-3, 1000));
        assertRejected("diameter", WheelPair.Flywheel.of(Double.NaN, 1000));
        assertRejected("diameter", WheelPair.Flywheel.of(Double.POSITIVE_INFINITY, 1000));
        assertRejected("rim", WheelPair.Flywheel.of(1e300, 1e300));
    }

    @Test
    public void aFlywheelThatSlowsHasAMomentOfInertia() {
        assertRejected("moment of inertia", WheelPair.Inertia.Slows.of(0));
        assertRejected("moment of inertia", WheelPair.Inertia.Slows.of(-1e-4));
        assertRejected("moment of inertia", WheelPair.Inertia.Slows.of(Double.NaN));
        assertRejected("moment of inertia", WheelPair.Inertia.Slows.of(Double.POSITIVE_INFINITY));
    }

    @Test
    public void anEfficiencyIsMoreThanNothingAndAtMostAll() {
        Asked asked = new Asked();
        asked.speedEfficiency = 0;
        assertRejected("speed efficiency", asked.pair());
        asked.speedEfficiency = 1.0000001;
        assertRejected("speed efficiency", asked.pair());
        asked.speedEfficiency = Double.NaN;
        assertRejected("speed efficiency", asked.pair());
        asked.speedEfficiency = 1;
        asked.spinEfficiency = 0;
        assertRejected("spin efficiency", asked.pair());
        asked.spinEfficiency = 1.5;
        assertRejected("spin efficiency", asked.pair());
        asked.spinEfficiency = Double.NaN;
        assertRejected("spin efficiency", asked.pair());
    }

    @Test
    public void theWheelsPressIntoThePollenByLessThanItIsAcross() {
        Asked asked = new Asked();
        asked.compression = -0.1;
        assertRejected("press", asked.pair());
        asked.compression = Double.NaN;
        assertRejected("press", asked.pair());
        asked.compression = POLLEN_DIAMETER;
        assertRejected("less than", asked.pair());
        asked.compression = 3;
        assertRejected("less than", asked.pair());
        asked.compression = Math.nextDown(POLLEN_DIAMETER);
        assertTrue(asked.pair() instanceof Checked.Ok);
    }

    @Test
    public void aLaunchLeavesFromSomewhereAtSomeAngleOnARobotMovingSomehow() {
        Asked asked = new Asked();
        asked.exit = new Vec3(Double.NaN, 0, 14);
        assertRejected("exit", asked.pair());
        asked.exit = new Vec3(0, 0, Double.POSITIVE_INFINITY);
        assertRejected("exit", asked.pair());
        asked.exit = EXIT;
        asked.yaw = Double.NaN;
        assertRejected("yaw", asked.pair());
        asked.yaw = 0;
        asked.pitch = Double.POSITIVE_INFINITY;
        assertRejected("pitch", asked.pair());
        asked.pitch = 0;
        asked.roll = Double.NEGATIVE_INFINITY;
        assertRejected("roll", asked.pair());
        asked.roll = 0;
        asked.robot = new Vec2(Double.NaN, 0);
        assertRejected("robot", asked.pair());
    }

    @Test
    public void aLaunchTooFastToCountIsRejected() {
        Asked asked = new Asked();
        asked.diameterOne = 1;
        asked.rpmOne = 1e300;
        asked.rpmTwo = 0;
        asked.compression = Math.nextDown(POLLEN_DIAMETER);
        assertRejected("finite", asked.pair());
        asked.inertia = slows(1);
        assertRejected("finite", asked.pair());
    }

    @Test
    public void wheelsAtRestThrowNothingAndDoNotSpinIt() {
        Asked asked = new Asked();
        asked.rpmOne = 0;
        asked.rpmTwo = 0;
        asked.inertia = slows(1e-4);

        WheelPair.Launched launched = asked.launched();

        assertExactly(Vec3.zero(), launched.velocity());
        assertEquals(WheelPair.SpinKind.NONE, launched.kind());
        assertEquals(0, launched.spinRpm(), 0);
    }
}
