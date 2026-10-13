package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AirTest {
    private static final double METRES_PER_INCH = 0.0254;
    private static final double GRAVITY = Flight.GRAVITY_IN_PER_S2;
    private static final double DENSITY = 1.2041;
    private static final double DRAG = 0.45;
    private static final double DRAG_PER_SPIN = 0.15;
    private static final double LIFT_SLOPE = 2;
    private static final double LIFT_LIMIT = 0.4;
    private static final double SPIN_DOWN = 0.012;
    private static final double ADDED_MASS = 0.5;
    private static final double RADIUS = 1.4;
    private static final double KILOGRAMS = 24.9 / 1000;
    private static final double INERTIA = 0.634;
    private static final Ball POLLEN = Valid.value(Ball.of(2 * RADIUS, 24.9, INERTIA));
    private static final Vec3 CALM = Vec3.zero();

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static double density(Checked<Double> checked) {
        return Valid.value(checked);
    }

    private static void assertVec(Vec3 expected, Vec3 actual, double delta) {
        assertEquals("x of " + actual, expected.x(), actual.x(), delta);
        assertEquals("y of " + actual, expected.y(), actual.y(), delta);
        assertEquals("z of " + actual, expected.z(), actual.z(), delta);
    }

    private static Air air(Vec3 wind, double drag, double dragPerSpin) {
        return Valid.value(Air.of(DENSITY, wind, drag, dragPerSpin, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
    }

    private static Air still() {
        return air(CALM, DRAG, DRAG_PER_SPIN);
    }

    private static Air windy(Vec3 wind) {
        return air(wind, DRAG, DRAG_PER_SPIN);
    }

    private static Air dragless() {
        return air(CALM, 0, 0);
    }

    private static double size(Vec3 v) {
        return Math.sqrt(v.dot(v));
    }

    private static Vec3 acrossTheFlight(Vec3 spin, Vec3 velocity) {
        return spin.minus(velocity.times(spin.dot(velocity) / velocity.dot(velocity)));
    }

    private static double radiusMetres() {
        return RADIUS * METRES_PER_INCH;
    }

    private static double areaSquareMetres() {
        return Math.PI * radiusMetres() * radiusMetres();
    }

    private static double displacedKilograms() {
        return DENSITY * 4.0 / 3 * Math.PI * radiusMetres() * radiusMetres() * radiusMetres();
    }

    private static double movingKilograms() {
        return KILOGRAMS + ADDED_MASS * displacedKilograms();
    }

    private static double fallingGravity() {
        return GRAVITY * (KILOGRAMS - displacedKilograms()) / movingKilograms();
    }

    private static double perInch() {
        return DENSITY * areaSquareMetres() / (2 * movingKilograms()) * METRES_PER_INCH;
    }

    private static double liftCoefficientAt(double s) {
        return LIFT_SLOPE * s / (1 + LIFT_SLOPE * s / LIFT_LIMIT);
    }

    private static double spinDownPerInch() {
        return SPIN_DOWN * DENSITY * areaSquareMetres() / (INERTIA * KILOGRAMS) * METRES_PER_INCH;
    }

    private static Vec3 airsPush(Air.Motion motion) {
        return motion.acceleration().plus(new Vec3(0, 0, fallingGravity()));
    }

    private static Vec3 spinsPush(Vec3 spin) {
        Vec3 velocity = new Vec3(300, 0, 0);
        Vec3 spun = still().motion(POLLEN, velocity, spin).acceleration();
        return spun.minus(still().motion(POLLEN, velocity, Vec3.zero()).acceleration());
    }

    @Test
    public void dryAirAtTwentyDegreesAtSeaLevelIsTheStandardDensity() {
        assertEquals(1.2041, density(Air.densityOf(20, 0, 0)), 1e-4);
    }

    @Test
    public void humidAirIsLighterThanDryAir() {
        double humid = density(Air.densityOf(20, 0, 50));

        assertEquals(1.1988, humid, 1e-4);
        assertTrue(humid < density(Air.densityOf(20, 0, 0)));
    }

    @Test
    public void airHigherUpIsThinner() {
        assertEquals(0.991, density(Air.densityOf(20, 1609, 0)), 1e-3);
    }

    @Test
    public void airHoldsNoMoreVapourThanItsPressureSoHotWetAirIsSteamAndNeverNegative() {
        double kelvin = 200 + 273.15;

        assertEquals(101325 / (461.495 * kelvin), density(Air.densityOf(200, 0, 100)), 1e-9);
    }

    @Test
    public void airTooColdToHoldVapourIsDryWhateverTheHumidity() {
        double kelvin = -250 + 273.15;

        assertEquals(101325 / (287.058 * kelvin), density(Air.densityOf(-250, 0, 100)), 1e-9);
    }

    @Test
    public void aDensityIsOfAirAboveAbsoluteZeroWithinTheAtmosphereAtAHumidityThatIsAPercentage() {
        assertRejected("absolute zero", Air.densityOf(-273.15, 0, 0));
        assertRejected("absolute zero", Air.densityOf(-300, 0, 0));
        assertRejected("absolute zero", Air.densityOf(Double.NaN, 0, 0));
        assertRejected("absolute zero", Air.densityOf(Double.POSITIVE_INFINITY, 0, 0));
        assertRejected("elevation", Air.densityOf(20, 44330, 0));
        assertRejected("elevation", Air.densityOf(20, Double.NaN, 0));
        assertRejected("elevation", Air.densityOf(20, Double.NEGATIVE_INFINITY, 0));
        assertRejected("humidity", Air.densityOf(20, 0, -1));
        assertRejected("humidity", Air.densityOf(20, 0, 100.5));
        assertRejected("humidity", Air.densityOf(20, 0, Double.NaN));
    }

    @Test
    public void conditionsSoFarOutThatNoFiniteDensityComesOfThemAreRejected() {
        assertRejected("no finite, positive density", Air.densityOf(1e308, 0, 0));
        assertRejected("no finite, positive density", Air.densityOf(20, -1e300, 0));
    }

    @Test
    public void theAirIsAsDenseAsItWasMade() {
        assertEquals(DENSITY, still().density(), 0);
    }

    @Test
    public void aStillPollenInStillAirFallsAtGravityLessWhatTheAirItDisplacesHoldsUp() {
        Air.Motion motion = still().motion(POLLEN, Vec3.zero(), Vec3.zero());

        assertEquals(0, motion.acceleration().x(), 0);
        assertEquals(0, motion.acceleration().y(), 0);
        assertEquals(-fallingGravity(), motion.acceleration().z(), 1e-12);
        assertTrue(fallingGravity() < GRAVITY);
        assertEquals(0.986, fallingGravity() / GRAVITY, 0.002);
        assertVec(Vec3.zero(), motion.spinRate(), 0);
    }

    @Test
    public void aPollenFallingAtItsTerminalSpeedNoLongerSpeedsUp() {
        double terminal = Math.sqrt(fallingGravity() / (perInch() * DRAG));
        Air air = still();

        double atTerminal = air.motion(POLLEN, new Vec3(0, 0, -terminal), Vec3.zero())
                .acceleration()
                .z();
        double slower = air.motion(POLLEN, new Vec3(0, 0, -0.9 * terminal), Vec3.zero())
                .acceleration()
                .z();
        double faster = air.motion(POLLEN, new Vec3(0, 0, -1.1 * terminal), Vec3.zero())
                .acceleration()
                .z();

        assertEquals(0, atTerminal, 1e-9 * GRAVITY);
        assertTrue("still speeding up at " + slower, slower < 0);
        assertTrue("slowing down at " + faster, faster > 0);
        assertEquals(15.0, terminal * METRES_PER_INCH, 0.2);
    }

    @Test
    public void dragPushesAgainstTheVelocityThroughTheAir() {
        Vec3 velocity = new Vec3(200, -100, 50);
        double speed = size(velocity);

        Vec3 push = airsPush(still().motion(POLLEN, velocity, Vec3.zero()));

        assertVec(velocity.times(-perInch() * DRAG * speed), push, 1e-9);
    }

    @Test
    public void aWindPushesAStillPollenDownwind() {
        Air.Motion motion = windy(new Vec3(100, 0, 0)).motion(POLLEN, Vec3.zero(), Vec3.zero());

        assertVec(new Vec3(perInch() * DRAG * 100 * 100, 0, -fallingGravity()), motion.acceleration(), 1e-9);
        assertTrue(motion.acceleration().x() > 0);
    }

    @Test
    public void aPollenMovingWithTheWindFeelsNoAir() {
        Vec3 wind = new Vec3(100, 50, 0);

        Air.Motion motion = windy(wind).motion(POLLEN, wind, new Vec3(0, -300, 0));

        assertVec(new Vec3(0, 0, -fallingGravity()), motion.acceleration(), 1e-12);
        assertVec(Vec3.zero(), motion.spinRate(), 0);
    }

    @Test
    public void backspinLiftsAPollen() {
        Vec3 push = spinsPush(new Vec3(0, -300, 0));

        assertTrue("lifted by " + push, push.z() > 0);
        assertEquals(0, push.y(), 1e-12);
    }

    @Test
    public void topspinPushesAPollenDown() {
        Vec3 push = spinsPush(new Vec3(0, 300, 0));

        assertTrue("lifted by " + push, push.z() < 0);
        assertEquals(0, push.y(), 1e-12);
    }

    @Test
    public void spinAboutUpCurvesAPollenToItsLeft() {
        Vec3 push = spinsPush(new Vec3(0, 0, 300));

        assertTrue("curved by " + push, push.y() > 0);
        assertEquals(0, push.z(), 1e-12);
    }

    @Test
    public void spinAlongTheFlightNeitherLiftsNorAddsDrag() {
        Vec3 velocity = new Vec3(300, 0, 0);

        Air.Motion rifled = still().motion(POLLEN, velocity, new Vec3(300, 0, 0));
        Air.Motion unspun = still().motion(POLLEN, velocity, Vec3.zero());

        assertVec(unspun.acceleration(), rifled.acceleration(), 1e-12);
        assertTrue("dragged by " + rifled.acceleration(), rifled.acceleration().x() < 0);
    }

    @Test
    public void onlyTheSpinAcrossTheFlightPushes() {
        Vec3 velocity = new Vec3(300, 0, 0);

        Air.Motion tilted = still().motion(POLLEN, velocity, new Vec3(150, 0, 300));
        Air.Motion across = still().motion(POLLEN, velocity, new Vec3(0, 0, 300));

        assertVec(across.acceleration(), tilted.acceleration(), 1e-9);
        assertTrue("curved by " + tilted.acceleration(), tilted.acceleration().y() > 0);
    }

    @Test
    public void liftIsKTimesTheLiftCoefficientTimesTheSquaredSpeedAlongTheSpinCrossTheFlight() {
        Vec3 velocity = new Vec3(200, 100, -50);
        Vec3 spin = new Vec3(40, -250, 120);
        double speed = size(velocity);
        double s = RADIUS * size(acrossTheFlight(spin, velocity)) / speed;
        Vec3 sideways = spin.cross(velocity);
        Vec3 lift = sideways.times(perInch() * liftCoefficientAt(s) * speed * speed / size(sideways));

        Vec3 push = airsPush(dragless().motion(POLLEN, velocity, spin));

        assertVec(lift, push, 1e-9 * size(lift));
    }

    @Test
    public void spinAddsDragInProportionToTheSpinParameter() {
        Vec3 velocity = new Vec3(300, 0, 0);
        double s = RADIUS * 300 / 300;

        Vec3 push = airsPush(still().motion(POLLEN, velocity, new Vec3(0, -300, 0)));

        assertEquals(DRAG, still().dragCoefficient(0), 0);
        assertEquals(DRAG + DRAG_PER_SPIN * 0.3, still().dragCoefficient(0.3), 1e-15);
        assertEquals(-perInch() * (DRAG + DRAG_PER_SPIN * s) * 300 * 300, push.x(), 1e-9);
    }

    @Test
    public void theLiftCoefficientRisesAtTheLiftSlopeAndLevelsOffAtTheLiftLimit() {
        Air air = still();
        double small = 1e-9;

        assertEquals(0, air.liftCoefficient(0), 0);
        assertEquals(LIFT_SLOPE, (air.liftCoefficient(small) - air.liftCoefficient(0)) / small, 1e-6);
        assertEquals(liftCoefficientAt(0.5), air.liftCoefficient(0.5), 1e-15);
        assertEquals(LIFT_LIMIT, air.liftCoefficient(1e9), 1e-6);
        assertTrue(air.liftCoefficient(1e9) < LIFT_LIMIT);
    }

    @Test
    public void raisingTheLiftSlopeOrTheLiftLimitRaisesLift() {
        Air steeper = Valid.value(Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, 3, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        Air higher = Valid.value(Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, 0.6, SPIN_DOWN, ADDED_MASS));

        assertTrue(steeper.liftCoefficient(0.05) > still().liftCoefficient(0.05));
        assertTrue(higher.liftCoefficient(1) > still().liftCoefficient(1));
    }

    @Test
    public void theCoefficientsGoByTheSpinParametersSizeAndLiftTurnsWithTheSpin() {
        Air air = still();

        assertEquals(air.dragCoefficient(0.3), air.dragCoefficient(-0.3), 0);
        assertEquals(-air.liftCoefficient(0.3), air.liftCoefficient(-0.3), 0);
    }

    @Test
    public void theSpinParameterIsTheSpinAcrossTheFlightAtTheSurfaceOverTheSpeedThroughTheAir() {
        Vec3 wind = new Vec3(30, -20, 0);
        Vec3 velocity = new Vec3(230, 80, -50);
        Vec3 spin = new Vec3(40, -250, 120);
        Vec3 through = velocity.minus(wind);

        double s = windy(wind).spinParameter(POLLEN, velocity, spin);

        assertEquals(RADIUS * size(acrossTheFlight(spin, through)) / size(through), s, 1e-12);
        assertEquals(0, still().spinParameter(POLLEN, Vec3.zero(), spin), 0);
        assertEquals(0, windy(wind).spinParameter(POLLEN, wind, spin), 0);
    }

    @Test
    public void withNoAirFlowingPastItASpinningPollenOnlyFalls() {
        Air.Motion motion = still().motion(POLLEN, Vec3.zero(), new Vec3(0, -300, 40));

        assertTrue(motion.acceleration().isFinite());
        assertVec(new Vec3(0, 0, -fallingGravity()), motion.acceleration(), 1e-12);
        assertVec(Vec3.zero(), motion.spinRate(), 0);
    }

    @Test
    public void theAirsPushFadesToNothingAsTheSpeedThroughItDoes() {
        Vec3 push = airsPush(still().motion(POLLEN, new Vec3(1e-6, 0, 0), new Vec3(0, -300, 40)));

        assertTrue(push.isFinite());
        assertEquals(0, size(push), 1e-6);
    }

    @Test
    public void spinSlowsInProportionToItselfAndToTheSpeedThroughTheAir() {
        Vec3 spin = new Vec3(10, -300, 50);

        Air.Motion motion = still().motion(POLLEN, new Vec3(300, 0, 0), spin);

        assertVec(spin.times(-spinDownPerInch() * 300), motion.spinRate(), 1e-10);
    }

    @Test
    public void atTenMetresASecondAPollensSpinTakesAboutTwentySevenSecondsToFallByAFactorOfE() {
        Vec3 spin = new Vec3(0, -300, 0);
        double tenMetresASecond = 10 / METRES_PER_INCH;

        Air.Motion motion = still().motion(POLLEN, new Vec3(tenMetresASecond, 0, 0), spin);

        assertEquals(27.5, size(spin) / size(motion.spinRate()), 0.5);
    }

    @Test
    public void aVacuumHasOnlyGravity() {
        Air vacuum = Air.vacuum();

        Air.Motion motion = vacuum.motion(POLLEN, new Vec3(300, 100, 200), new Vec3(0, -300, 50));

        assertVec(new Vec3(0, 0, -GRAVITY), motion.acceleration(), 0);
        assertVec(Vec3.zero(), motion.spinRate(), 0);
        assertEquals(0, vacuum.density(), 0);
        assertEquals(0, vacuum.dragCoefficient(0.5), 0);
        assertEquals(0, vacuum.liftCoefficient(0.5), 0);
        assertVec(
                new Vec3(0, 0, -GRAVITY),
                vacuum.motion(POLLEN, Vec3.zero(), Vec3.zero()).acceleration(),
                0);
    }

    @Test
    public void airMayBeThinWindyAndWithoutDragLiftOrSpinDown() {
        Air air = Valid.value(Air.of(0.5, new Vec3(30, -20, 0), 0, 0, 0, 0.1, 0, 0));

        assertEquals(0.5, air.density(), 0);
        assertEquals(0, air.liftCoefficient(0.5), 0);
    }

    @Test
    public void airHasAPositiveDensityAFiniteWindAndCoefficientsThatAreNeverNegative() {
        double nan = Double.NaN, infinite = Double.POSITIVE_INFINITY;
        assertRejected("density", Air.of(0, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "density", Air.of(nan, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "density", Air.of(infinite, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "wind",
                Air.of(
                        DENSITY,
                        new Vec3(nan, 0, 0),
                        DRAG,
                        DRAG_PER_SPIN,
                        LIFT_SLOPE,
                        LIFT_LIMIT,
                        SPIN_DOWN,
                        ADDED_MASS));
        assertRejected(
                "wind",
                Air.of(
                        DENSITY,
                        new Vec3(0, 0, infinite),
                        DRAG,
                        DRAG_PER_SPIN,
                        LIFT_SLOPE,
                        LIFT_LIMIT,
                        SPIN_DOWN,
                        ADDED_MASS));
        assertRejected(
                "drag coefficient",
                Air.of(DENSITY, CALM, -0.1, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "drag coefficient",
                Air.of(DENSITY, CALM, nan, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "each unit of spin parameter adds",
                Air.of(DENSITY, CALM, DRAG, -0.1, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "each unit of spin parameter adds",
                Air.of(DENSITY, CALM, DRAG, infinite, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected("lift slope", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, -1, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "lift slope", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, nan, LIFT_LIMIT, SPIN_DOWN, ADDED_MASS));
        assertRejected("lift limit", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, 0, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "lift limit", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, -0.4, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "lift limit", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, infinite, SPIN_DOWN, ADDED_MASS));
        assertRejected(
                "spin-down", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, -0.001, ADDED_MASS));
        assertRejected(
                "spin-down", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, nan, ADDED_MASS));
        assertRejected(
                "added-mass", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, -0.5));
        assertRejected(
                "added-mass", Air.of(DENSITY, CALM, DRAG, DRAG_PER_SPIN, LIFT_SLOPE, LIFT_LIMIT, SPIN_DOWN, infinite));
    }
}
