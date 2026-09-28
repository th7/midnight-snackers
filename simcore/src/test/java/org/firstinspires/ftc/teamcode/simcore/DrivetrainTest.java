package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DrivetrainTest {
    private static final double KS_VOLTS = 1;
    private static final double KV_VOLT_SECONDS_PER_TICK = 0.0001;
    private static final double KA_VOLT_SECONDS_SQUARED_PER_TICK = 0.00005;
    private static final double IN_PER_TICK = 0.0005;
    private static final double VOLTS = 12;
    private static final double DELTA = 1e-9;

    private static final Feedforward TUNED =
            Valid.value(Feedforward.of(KS_VOLTS, KV_VOLT_SECONDS_PER_TICK, KA_VOLT_SECONDS_SQUARED_PER_TICK));
    private static final Drivetrain EXACT =
            Valid.value(Drivetrain.of(TUNED, IN_PER_TICK, PerWheel.all(Noise.Motor.tuned())));

    private static final Seconds STEP = Valid.value(Seconds.of(0.005));

    private static double freeSpeed(double volts) {
        return (volts - KS_VOLTS) / KV_VOLT_SECONDS_PER_TICK * IN_PER_TICK;
    }

    private static Drivetrain.Setting forward(double power) {
        return new Drivetrain.Setting(Sense.FORWARD, Power.clamped(power), ZeroPower.BRAKE);
    }

    private static double leftFront(
            Drivetrain drivetrain, Drivetrain.Setting setting, double velocity, Traction traction) {
        return Valid.value(drivetrain.accelerations(
                        PerWheel.all(forward(0)).with(Wheel.LEFT_FRONT, setting),
                        VOLTS,
                        PerWheel.all(0.0).with(Wheel.LEFT_FRONT, velocity),
                        STEP,
                        traction))
                .leftFront();
    }

    private static double leftFront(Drivetrain.Setting setting, double velocity) {
        return leftFront(EXACT, setting, velocity, Traction.unlimited());
    }

    @Test
    public void fromRestFullPowerPushesAsHardAsTheBatteryAboveStaticFrictionAllows() {
        double expected = (VOLTS - KS_VOLTS) / KA_VOLT_SECONDS_SQUARED_PER_TICK * IN_PER_TICK;

        assertEquals(expected, leftFront(forward(1), 0), DELTA);
        assertEquals(
                "a half-power push is less by half the battery",
                expected - 6 / KA_VOLT_SECONDS_SQUARED_PER_TICK * IN_PER_TICK,
                leftFront(forward(0.5), 0),
                DELTA);
    }

    @Test
    public void atTheSpeedKsAndKvGiveThePushIsSpent() {
        assertEquals(0, leftFront(forward(1), freeSpeed(VOLTS)), 1e-6);
        assertTrue("slower, it pushes on", leftFront(forward(1), freeSpeed(VOLTS) - 1) > 0);
        assertTrue("faster, it holds back", leftFront(forward(1), freeSpeed(VOLTS) + 1) < 0);
    }

    @Test
    public void powerBelowStaticFrictionLeavesAWheelAtRestAtRest() {
        double belowKs = 0.9 * KS_VOLTS / VOLTS;

        assertEquals(0, leftFront(forward(belowKs), 0), 0);
        assertEquals(0, leftFront(forward(-belowKs), 0), 0);
    }

    @Test
    public void aWheelCreepingBelowStaticFrictionIsStoppedWithinTheStep() {
        double creeping = 0.01;

        assertEquals(-creeping / STEP.value(), leftFront(forward(0), creeping), DELTA);
        assertEquals(creeping / STEP.value(), leftFront(forward(0), -creeping), DELTA);
    }

    @Test
    public void inNoTimeAtAllAWheelAtRestIsGivenNothing() {
        double given = Valid.value(EXACT.accelerations(
                        PerWheel.all(forward(0)), VOLTS, PerWheel.all(0.0), Seconds.zero(), Traction.unlimited()))
                .leftFront();

        assertEquals(0, given, 0);
    }

    @Test
    public void aRollingWheelAtZeroPowerIsHeldBackHarderBrakingThanFloating() {
        double rolling = 30;
        Drivetrain.Setting braking = new Drivetrain.Setting(Sense.FORWARD, Power.none(), ZeroPower.BRAKE);
        Drivetrain.Setting floating = new Drivetrain.Setting(Sense.FORWARD, Power.none(), ZeroPower.FLOAT);

        double floats = leftFront(floating, rolling);
        double brakes = leftFront(braking, rolling);

        assertEquals(
                "floating, only static friction",
                -KS_VOLTS / KA_VOLT_SECONDS_SQUARED_PER_TICK * IN_PER_TICK,
                floats,
                DELTA);
        assertEquals(
                "braking, the motor's back EMF too",
                (-KS_VOLTS - KV_VOLT_SECONDS_PER_TICK * rolling / IN_PER_TICK)
                        / KA_VOLT_SECONDS_SQUARED_PER_TICK
                        * IN_PER_TICK,
                brakes,
                DELTA);
        assertTrue(brakes < floats);
    }

    @Test
    public void aRollingWheelAtZeroPowerWithNoZeroPowerBehaviorIsRefusedRatherThanGuessedAt() {
        Drivetrain.Setting unknown = new Drivetrain.Setting(Sense.FORWARD, Power.none(), ZeroPower.UNKNOWN);

        Checked<PerWheel<Double>> refused = EXACT.accelerations(
                PerWheel.all(forward(0)).with(Wheel.RIGHT_BACK, unknown),
                VOLTS,
                PerWheel.all(30.0),
                STEP,
                Traction.unlimited());

        String said = refused.fold(value -> "accepted " + value, rule -> rule);
        assertTrue(said, refused instanceof Checked.Rejected);
        assertTrue(said, said.contains("right back") && said.contains("zero power behavior UNKNOWN"));
    }

    @Test
    public void aWheelDrivenOrAtRestNeedsNoZeroPowerBehavior() {
        Drivetrain.Setting driven = new Drivetrain.Setting(Sense.FORWARD, Power.clamped(0.5), ZeroPower.UNKNOWN);
        Drivetrain.Setting resting = new Drivetrain.Setting(Sense.FORWARD, Power.none(), ZeroPower.UNKNOWN);

        assertEquals(leftFront(forward(0.5), 30), leftFront(driven, 30), 0);
        assertEquals(0, leftFront(resting, 0), 0);
    }

    @Test
    public void theFloorGivesOnlySoMuchTractionDrivingOrBraking() {
        Traction traction = Valid.value(Traction.of(40));

        assertEquals(40, leftFront(EXACT, forward(1), 0, traction), 0);
        assertEquals(-40, leftFront(EXACT, forward(-1), 0, traction), 0);
        assertEquals(-40, leftFront(EXACT, forward(0), 30, traction), 0);
        assertEquals(
                "a gentle push does not slip",
                leftFront(forward(1), freeSpeed(VOLTS) - 0.001),
                leftFront(EXACT, forward(1), freeSpeed(VOLTS) - 0.001, traction),
                0);
    }

    @Test
    public void aReversedMotorOrAMirroredMountPushesTheOtherWay() {
        Drivetrain.Setting reversed = new Drivetrain.Setting(Sense.REVERSE, Power.clamped(1), ZeroPower.BRAKE);
        PerWheel<Double> pushes = Valid.value(
                EXACT.accelerations(PerWheel.all(forward(1)), VOLTS, PerWheel.all(0.0), STEP, Traction.unlimited()));
        PerWheel<Double> reversedPushes = Valid.value(
                EXACT.accelerations(PerWheel.all(reversed), VOLTS, PerWheel.all(0.0), STEP, Traction.unlimited()));

        double ahead = leftFront(forward(1), 0);
        assertEquals(new PerWheel<>(ahead, ahead, -ahead, -ahead), pushes);
        assertEquals(new PerWheel<>(-ahead, -ahead, ahead, ahead), reversedPushes);
        assertEquals(Sense.FORWARD, Wheel.LEFT_FRONT.mounted());
        assertEquals(Sense.REVERSE, Wheel.RIGHT_BACK.mounted());
    }

    @Test
    public void aSaggingBatteryPushesLess() {
        double sagged = Valid.value(EXACT.accelerations(
                        PerWheel.all(forward(1)), 10, PerWheel.all(0.0), STEP, Traction.unlimited()))
                .leftFront();

        assertEquals((10 - KS_VOLTS) / KA_VOLT_SECONDS_SQUARED_PER_TICK * IN_PER_TICK, sagged, DELTA);
    }

    @Test
    public void aNoisyMotorIsItsTuningScaledByItsNoise() {
        Noise.Motor weaker = Valid.value(Noise.Motor.of(1, 1.1, 1));
        Drivetrain noisy = Valid.value(Drivetrain.of(
                TUNED, IN_PER_TICK, PerWheel.all(Noise.Motor.tuned()).with(Wheel.LEFT_FRONT, weaker)));

        double weakerFreeSpeed = (VOLTS - KS_VOLTS) / (1.1 * KV_VOLT_SECONDS_PER_TICK) * IN_PER_TICK;
        assertEquals(0, leftFront(noisy, forward(1), weakerFreeSpeed, Traction.unlimited()), 1e-6);
        Feedforward scaled = Valid.value(TUNED.scaledBy(weaker));
        assertEquals(KS_VOLTS, scaled.kSVolts(), 0);
        assertEquals(1.1 * KV_VOLT_SECONDS_PER_TICK, scaled.kVVoltSecondsPerTick(), 0);
        assertEquals(KA_VOLT_SECONDS_SQUARED_PER_TICK, scaled.kAVoltSecondsSquaredPerTick(), 0);
    }

    @Test
    public void aPowerIsWithinFullEitherWayAndOneThatIsNotANumberIsNoneAsTheHubTakesIt() {
        assertEquals(1, Power.clamped(3).value(), 0);
        assertEquals(-1, Power.clamped(-3).value(), 0);
        assertEquals(0.25, Power.clamped(0.25).value(), 0);
        assertEquals(0, Power.clamped(Double.NaN).value(), 0);
        assertEquals(0, Power.none().value(), 0);
    }

    @Test
    public void aTuningAndADrivetrainAreChecked() {
        assertTrue(Feedforward.of(-1, 1, 1) instanceof Checked.Rejected);
        assertTrue(Feedforward.of(1, -1, 1) instanceof Checked.Rejected);
        assertTrue(Feedforward.of(1, 1, 0) instanceof Checked.Rejected);
        assertTrue(Feedforward.of(1, 1, Double.NaN) instanceof Checked.Rejected);
        assertTrue(Feedforward.of(Double.POSITIVE_INFINITY, 1, 1) instanceof Checked.Rejected);
        assertTrue(Drivetrain.of(TUNED, 0, PerWheel.all(Noise.Motor.tuned())) instanceof Checked.Rejected);
        assertTrue(Drivetrain.of(TUNED, Double.NaN, PerWheel.all(Noise.Motor.tuned())) instanceof Checked.Rejected);
    }
}
