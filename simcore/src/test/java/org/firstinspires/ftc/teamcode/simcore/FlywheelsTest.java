package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FlywheelsTest {
    private static final MotorType HD_HEX = MotorType.ULTRAPLANETARY_HD_HEX;
    private static final double DELTA = 1e-9;

    private static Flywheels reginalds() {
        return Valid.value(RobotDescription.reginalds().flywheels());
    }

    @Test
    public void theFrontWheelLaunchesWhileItsMotorTurnsClockwiseSeenFromItsShaft() {
        Flywheels.Turning turning = reginalds().turning(Flywheels.Side.FRONT, -1400, HD_HEX, false);

        assertEquals(Flywheels.Way.LAUNCHING, turning.way());
        assertEquals(3000, turning.rpm(), DELTA);
        assertEquals(593.761011528, turning.surfaceInPerS(), 1e-6);
    }

    @Test
    public void theFrontWheelTurningCounterclockwiseSeenFromItsShaftTurnsTheWrongWay() {
        Flywheels.Turning turning = reginalds().turning(Flywheels.Side.FRONT, 1400, HD_HEX, false);

        assertEquals(Flywheels.Way.WRONG_WAY, turning.way());
        assertEquals(-3000, turning.rpm(), DELTA);
    }

    @Test
    public void theBackWheelLaunchesWhileItsMotorTurnsCounterclockwiseSeenFromItsShaft() {
        assertEquals(
                Flywheels.Way.LAUNCHING,
                reginalds().turning(Flywheels.Side.BACK, 1400, HD_HEX, false).way());
        assertEquals(
                Flywheels.Way.WRONG_WAY,
                reginalds().turning(Flywheels.Side.BACK, -1400, HD_HEX, false).way());
    }

    @Test
    public void aReversedMotorLaunchesTurningTheOtherWay() {
        assertEquals(
                Flywheels.Way.WRONG_WAY,
                reginalds().turning(Flywheels.Side.BACK, 1400, HD_HEX, true).way());
        assertEquals(
                Flywheels.Way.LAUNCHING,
                reginalds().turning(Flywheels.Side.FRONT, 1400, HD_HEX, true).way());
    }

    @Test
    public void aWheelTurningUnderAOneTwentiethOfItsMotorsFreeSpeedIsStill() {
        assertEquals(
                Flywheels.Way.STILL,
                reginalds().turning(Flywheels.Side.BACK, 140, HD_HEX, false).way());
        assertEquals(
                Flywheels.Way.STILL,
                reginalds().turning(Flywheels.Side.BACK, -140, HD_HEX, false).way());
        assertEquals(
                Flywheels.Way.LAUNCHING,
                reginalds().turning(Flywheels.Side.BACK, 141, HD_HEX, false).way());
    }

    @Test
    public void theMismatchIsHowMuchFasterTheFrontWheelsSurfaceMovesThanTheBacks() {
        Flywheels flywheels = reginalds();
        Flywheels.Turning front = flywheels.turning(Flywheels.Side.FRONT, -1400, HD_HEX, false);
        Flywheels.Turning back = flywheels.turning(Flywheels.Side.BACK, 1120, HD_HEX, false);

        assertEquals(118.752202306, Flywheels.mismatch(front, back), 1e-6);
        assertEquals(-118.752202306, Flywheels.mismatch(back, front), 1e-6);
    }

    @Test
    public void eachSideIsTheMechanismWhoseDirectionItTakes() {
        assertEquals(Mechanism.FLYWHEEL_FRONT, Flywheels.Side.FRONT.mechanism());
        assertEquals(Mechanism.FLYWHEEL_BACK, Flywheels.Side.BACK.mechanism());
    }

    @Test
    public void eachWheelSpinsUpOnTheFeedforwardTheDescriptionGivesIt() {
        Constants slowerBack = Valid.value(
                Constants.of(java.util.Map.of(Constant.FLYWHEEL_BACK_KA_VOLT_SECONDS_SQUARED_PER_TICK, 0.02)));
        Flywheels flywheels =
                Valid.value(RobotDescription.reginalds().with(slowerBack).flywheels());

        assertEquals(0.00713, flywheels.motor(Flywheels.Side.FRONT).kAVoltSecondsSquaredPerTick(), 0);
        assertEquals(0.02, flywheels.motor(Flywheels.Side.BACK).kAVoltSecondsSquaredPerTick(), 0);
    }

    @Test
    public void aWheelIsNeverAnythingButAPositiveFiniteSize() {
        Feedforward motor = Valid.value(Feedforward.of(0.5, 0.004, 0.007));
        Flywheel.TicksPerTurn ticks = Valid.value(Flywheel.TicksPerTurn.of(28));
        String said = Flywheels.of(motor, motor, ticks, 0, 3.78).fold(made -> "made " + made, rule -> rule);

        assertTrue(said, said.contains("a flywheel's diameter is a positive, finite number of inches, not 0"));
    }
}
