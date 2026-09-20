package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.junit.Test;

public class SimRobotNoiseTest {
    private static final double MAX_SPEED_IN_PER_S = (SimRobot.BATTERY_VOLTS - MecanumDrive.PARAMS.kSVolts)
            / MecanumDrive.PARAMS.kVVoltSecondsPerTick
            * MecanumDrive.PARAMS.inPerTick;

    private static final Pose2d OPEN = new Pose2d(-60, 0, 0);

    @Test
    public void weakerMotorsDriveSlower() {
        SimRobot sim = new SimRobot(SimNoise.NONE.withMotors(new SimNoise.Motor(1, 1.1, 1)));

        assertEquals(MAX_SPEED_IN_PER_S / 1.1, freeSpeed(sim), 0.03 * MAX_SPEED_IN_PER_S);
    }

    @Test
    public void aWeakerRightSideTurnsTheRobotRightUnderEqualPower() {
        SimNoise.Motor weaker = new SimNoise.Motor(1, 1.15, 1);
        SimRobot sim = new SimRobot(
                SimNoise.NONE.withMotor(SimNoise.RIGHT_FRONT, weaker).withMotor(SimNoise.RIGHT_BACK, weaker));
        robotDrive(sim, robotLocalizer(sim));
        sim.setPose(OPEN);

        setPowers(sim, 1, 1, 1, 1);
        sim.step(1.5);

        assertTrue(
                "heading=" + sim.pose().heading.toDouble(), sim.pose().heading.toDouble() < -0.05);
    }

    @Test
    public void theBatteryReadsFreshThenSagsUnderLoadAndDrainsWithTime() {
        SimRobot sim = new SimRobot(SimNoise.NONE.withBattery(new SimNoise.Battery(13.8, 0.2, 0.004)));
        robotDrive(sim, robotLocalizer(sim));

        assertEquals(13.8, sim.voltageSensor.getVoltage(), 1e-9);

        setPowers(sim, 1, 1, 1, 1);
        sim.step(0.005);
        assertEquals("all four at full power", 13.8 - 0.8 - 0.004 * 0.005, sim.voltageSensor.getVoltage(), 1e-9);

        setPowers(sim, 0, 0, 0, 0);
        sim.step(100);
        assertEquals("a hundred seconds in", 13.8 - 0.004 * 100.005, sim.voltageSensor.getVoltage(), 1e-9);
    }

    @Test
    public void aFresherBatteryDrivesFaster() {
        SimRobot fresh = new SimRobot(SimNoise.NONE.withBattery(new SimNoise.Battery(13.8, 0, 0)));

        double expected = (13.8 - MecanumDrive.PARAMS.kSVolts)
                / MecanumDrive.PARAMS.kVVoltSecondsPerTick
                * MecanumDrive.PARAMS.inPerTick;
        assertEquals(expected, freeSpeed(fresh), 0.03 * expected);
        assertTrue(freeSpeed(fresh) > freeSpeed(new SimRobot()) + 3);
    }

    @Test
    public void aWheelCannotAccelerateTheRobotHarderThanTraction() {
        double traction = 40;
        SimRobot sim = new SimRobot(SimNoise.NONE.withTraction(traction));
        robotDrive(sim, robotLocalizer(sim));
        sim.setPose(OPEN);

        setPowers(sim, 1, 1, 1, 1);
        sim.step(0.5);

        assertEquals("slipping from rest", traction * 0.5, speed(sim), 1.5);
        double unlimited = speedAfterHalfASecond(new SimRobot());
        assertTrue("with all the traction it wants: " + unlimited, unlimited > traction * 0.5 + 5);
    }

    @Test
    public void aWheelCannotBrakeTheRobotHarderThanTractionEither() {
        double traction = 40;
        SimRobot sim = new SimRobot(SimNoise.NONE.withTraction(traction));
        robotDrive(sim, robotLocalizer(sim));
        sim.setPose(OPEN);
        setPowers(sim, 1, 1, 1, 1);
        sim.step(0.5);
        double before = speed(sim);

        setPowers(sim, 0, 0, 0, 0);
        sim.step(0.25);

        assertTrue("slowed from " + before + " to " + speed(sim), speed(sim) >= before - traction * 0.25 - 0.5);
        assertTrue("but did slow", speed(sim) < before - 5);
    }

    @Test
    public void theDeadWheelsReadTheTrueMotionEvenWhileTheWheelsSlip() {
        SimRobot sim = new SimRobot(SimNoise.NONE.withTraction(40));
        Localizer localizer = robotLocalizer(sim);
        MecanumDrive drive = robotDrive(sim, localizer);
        localizer.update();

        setPowers(sim, 0.6, 1, 0.6, 1);
        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            localizer.update();
        }

        Pose2d truePose = sim.pose();
        Pose2d estimated = localizer.pose();
        assertEquals(truePose.position.x, estimated.position.x, 0.5);
        assertEquals(truePose.position.y, estimated.position.y, 0.5);
        assertEquals(truePose.heading.toDouble(), estimated.heading.toDouble(), 0.02);
    }

    @Test
    public void withoutNoiseSettingDownIsPlacingExactly() {
        SimRobot sim = new SimRobot();

        sim.setDown(new Pose2d(-56, -12, 0.3));

        assertEquals(-56, sim.pose().position.x, 1e-9);
        assertEquals(-12, sim.pose().position.y, 1e-9);
        assertEquals(0.3, sim.pose().heading.toDouble(), 1e-9);
    }

    @Test
    public void withNoiseSettingDownIsNearThePoseAndTheSameForTheSameSeed() {
        SimRobot sim = new SimRobot(SimNoise.seeded(11));
        SimRobot again = new SimRobot(SimNoise.seeded(11));

        sim.setDown(new Pose2d(-56, -12, 0.3));
        again.setDown(new Pose2d(-56, -12, 0.3));

        assertNotEquals(-56, sim.pose().position.x, 1e-6);
        assertEquals(-56, sim.pose().position.x, 5 * SimNoise.PLACEMENT_INCHES);
        assertEquals(-12, sim.pose().position.y, 5 * SimNoise.PLACEMENT_INCHES);
        assertEquals(0.3, sim.pose().heading.toDouble(), 5 * SimNoise.PLACEMENT_RADIANS);
        assertEquals(sim.pose(), again.pose());
        assertEquals("the IMU reads where it really is", sim.pose().heading.toDouble(), sim.imu.yawRadians, 0);
    }

    @Test
    public void aRobotSetDownBeyondAWallIsSetDownAgainstIt() {
        SimRobot sim = new SimRobot(SimNoise.seeded(12));

        sim.setDown(new Pose2d(SimPlacement.FIELD_SIZE_IN, 0, 0));

        assertEquals(SimPlacement.onTheField(sim.pose()), sim.pose());
        assertTrue(sim.pose().position.x < SimPlacement.FIELD_SIZE_IN / 2);
    }

    private static double freeSpeed(SimRobot sim) {
        robotDrive(sim, robotLocalizer(sim));
        sim.setPose(OPEN);
        setPowers(sim, 1, 1, 1, 1);
        sim.step(2.0);
        double before = sim.pose().position.x;
        sim.step(0.5);
        return (sim.pose().position.x - before) / 0.5;
    }

    private static double speedAfterHalfASecond(SimRobot sim) {
        robotDrive(sim, robotLocalizer(sim));
        sim.setPose(OPEN);
        setPowers(sim, 1, 1, 1, 1);
        sim.step(0.5);
        return speed(sim);
    }

    private static double speed(SimRobot sim) {
        double x0 = sim.pose().position.x;
        sim.step(0.005);
        return (sim.pose().position.x - x0) / 0.005;
    }

    private static Localizer robotLocalizer(SimRobot sim) {
        return new Localizer(
                sim.rightBack, sim.leftFront, () -> sim.imu, new Pose2d(0, 0, 0), sim::nanoTime, Prints.NOWHERE);
    }

    private static MecanumDrive robotDrive(SimRobot sim, Localizer localizer) {
        return new MecanumDrive(
                new Wheels(sim.leftFront, sim.leftBack, sim.rightBack, sim.rightFront),
                () -> sim.imu,
                sim.voltageSensor,
                localizer,
                sim::nanoTime);
    }

    private static void setPowers(
            SimRobot sim, double leftFront, double rightFront, double leftBack, double rightBack) {
        sim.leftFront.setPower(leftFront);
        sim.rightFront.setPower(rightFront);
        sim.leftBack.setPower(leftBack);
        sim.rightBack.setPower(rightBack);
    }
}
