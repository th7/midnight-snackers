package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.nugget.NuggetHardware;
import org.firstinspires.ftc.nugget.NuggetOpMode;
import org.firstinspires.ftc.nugget.NuggetTeleOp;
import org.firstinspires.ftc.nugget.TankLocalizer;
import org.firstinspires.ftc.nugget.Trajectories;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Test;

public class SimNuggetTest {
    private static final Pose2d CLEAR_OF_EVERYTHING = new Pose2d(-60, 0, 0);

    private static final Pose2d ROOM_TO_TURN = new Pose2d(-40, 0, 0);

    private static final double EXACTLY = 1e-9;

    private final SimRobot nugget = new SimRobot(TeamRobot.NUGGET, SimNoise.NONE, Constants.defaults());

    public static final class NeitherReversed extends NuggetOpMode {
        @Override
        protected NuggetHardware hardware() {
            return super.hardware();
        }

        @Override
        public void init() {
            hardware().left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware().right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        @Override
        public void loop() {
            hardware().left.setPower(1);
            hardware().right.setPower(1);
        }
    }

    public static final class CountingTeleOp extends NuggetTeleOp {
        NuggetHardware devices() {
            return hardware();
        }
    }

    public static final class LocalizingTeleOp extends NuggetTeleOp {
        TankLocalizer localizer;

        @Override
        public void init() {
            super.init();
            localizer = new TankLocalizer(hardware(), Trajectories.PARAMS, new Pose2d(0, 0, 0));
        }

        @Override
        public void loop() {
            localizer.loop();
            super.loop();
        }
    }

    private static <T extends OpMode> T started(SimRobot sim, T opMode) {
        sim.wire(opMode);
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.start();
        return opMode;
    }

    private static void drive(SimRobot sim, OpMode opMode, double seconds) {
        long loops = Math.round(seconds / SimRunner.LOOP_SECONDS);
        for (long loop = 0; loop < loops; loop++) {
            opMode.loop();
            sim.step(SimRunner.LOOP_SECONDS);
        }
    }

    @Test
    public void theLeftStickPushedForwardDrivesNuggetStraightAhead() {
        NuggetTeleOp teleOp = started(nugget, new NuggetTeleOp());
        nugget.setPose(CLEAR_OF_EVERYTHING);
        teleOp.gamepad1.left_stick_y = -1;

        drive(nugget, teleOp, 1);

        Pose2d pose = nugget.pose();
        assertTrue("a foot or more ahead in a second: " + pose, pose.position.x > CLEAR_OF_EVERYTHING.position.x + 12);
        assertEquals("and not an inch aside", 0, pose.position.y, 0.01);
        assertEquals("still facing ahead", 0, pose.heading.toDouble(), 0.001);
    }

    @Test
    public void theRightStickPushedLeftTurnsNuggetCounterclockwiseWhereItStands() {
        NuggetTeleOp teleOp = started(nugget, new NuggetTeleOp());
        nugget.setPose(ROOM_TO_TURN);
        teleOp.gamepad1.right_stick_x = -1;

        drive(nugget, teleOp, 0.5);

        Pose2d pose = nugget.pose();
        assertTrue("turned counterclockwise: " + pose, pose.heading.toDouble() > 0.3);
        assertEquals("where it stood", ROOM_TO_TURN.position.x, pose.position.x, 0.01);
        assertEquals("where it stood", ROOM_TO_TURN.position.y, pose.position.y, 0.01);
    }

    @Test
    public void nuggetsMotorsAreMountedMirrorImageSoWithNeitherReversedItSpinsRatherThanDrives() {
        NeitherReversed opMode = started(nugget, new NeitherReversed());
        nugget.setPose(ROOM_TO_TURN);

        drive(nugget, opMode, 0.5);

        Pose2d pose = nugget.pose();
        assertTrue("the left side drives back and the right ahead: " + pose, pose.heading.toDouble() > 0.3);
        assertEquals("so it goes nowhere", ROOM_TO_TURN.position.x, pose.position.x, 0.01);
    }

    @Test
    public void aSeededNuggetIsItsOwnImperfectRobotAndStillCannotSlideSideways() {
        SimRobot seeded = new SimRobot(TeamRobot.NUGGET, Noise.seeded(7, Constants.defaults()), Constants.defaults());
        NuggetTeleOp teleOp = started(seeded, new NuggetTeleOp());
        seeded.setPose(CLEAR_OF_EVERYTHING);
        teleOp.gamepad1.left_stick_y = -1;

        for (int loop = 0; loop < 50; loop++) {
            Pose2d before = seeded.pose();
            teleOp.loop();
            seeded.step(SimRunner.LOOP_SECONDS);
            double aside = seeded.pose().minus(before).line.y;
            assertEquals("on loop " + loop + " it moved aside", 0, aside, 1e-3);
        }
        assertTrue("and it drove: " + seeded.pose(), seeded.pose().position.x > CLEAR_OF_EVERYTHING.position.x + 12);
    }

    @Test
    public void nuggetsDriveMotorsCountHowFarTheirSideHasGoneAhead() {
        CountingTeleOp teleOp = started(nugget, new CountingTeleOp());
        nugget.setPose(CLEAR_OF_EVERYTHING);
        teleOp.gamepad1.left_stick_y = -1;

        drive(nugget, teleOp, 1);

        double ticks = (nugget.pose().position.x - CLEAR_OF_EVERYTHING.position.x) / MecanumDrive.PARAMS.inPerTick;
        assertTrue("it drove: " + nugget.pose(), ticks > 12 / MecanumDrive.PARAMS.inPerTick);
        assertEquals(ticks, teleOp.devices().left.getCurrentPosition(), 2);
        assertEquals(ticks, teleOp.devices().right.getCurrentPosition(), 2);
        assertTrue("and each says it is turning", teleOp.devices().left.getVelocity() > 0);
        assertEquals(teleOp.devices().left.getVelocity(), teleOp.devices().right.getVelocity(), 20);
    }

    @Test
    public void turningCounterclockwiseCountsTheLeftSideBackAndTheRightAheadByTheSimulatedTrack() {
        CountingTeleOp teleOp = started(nugget, new CountingTeleOp());
        nugget.setPose(ROOM_TO_TURN);
        teleOp.gamepad1.right_stick_x = -1;

        drive(nugget, teleOp, 0.5);

        double ticks = nugget.pose().heading.toDouble() * MecanumDrive.PARAMS.trackWidthTicks;
        assertTrue("it turned: " + nugget.pose(), ticks > 0);
        assertEquals(-ticks, teleOp.devices().left.getCurrentPosition(), 2);
        assertEquals(ticks, teleOp.devices().right.getCurrentPosition(), 2);
    }

    @Test
    public void aDriveMotorCountsTheWayItTurnsSoOneNotReversedCountsAheadAsItsSideGoesBack() {
        NeitherReversed opMode = started(nugget, new NeitherReversed());
        nugget.setPose(ROOM_TO_TURN);

        drive(nugget, opMode, 0.5);

        assertTrue(
                "it spun counterclockwise, the left side going back: " + nugget.pose(),
                nugget.pose().heading.toDouble() > 0.3);
        assertTrue(opMode.hardware().left.getCurrentPosition() > 0);
        assertTrue(opMode.hardware().right.getCurrentPosition() > 0);
    }

    @Test
    public void aNuggetOpModeIsHandedTheSimulatorsClockBatteryAndDashboard() {
        CountingTeleOp teleOp = started(nugget, new CountingTeleOp());

        drive(nugget, teleOp, 0.5);

        assertEquals(nugget.nanoTime(), teleOp.devices().nanoClock.getAsLong());
        assertTrue("half a second on", nugget.nanoTime() >= 490_000_000L);
        assertSame(nugget.voltageSensor, teleOp.devices().voltageSensor);
        assertSame(nugget.dashboard, teleOp.devices().dashboard);
    }

    @Test
    public void nuggetBelievesItIsWhereTheSimulatorHasTakenIt() {
        Pose2d start = new Pose2d(-48, -48, 0);
        LocalizingTeleOp teleOp = started(nugget, new LocalizingTeleOp());
        nugget.setPose(start);

        teleOp.gamepad1.left_stick_y = -0.5f;
        drive(nugget, teleOp, 1);
        teleOp.gamepad1.left_stick_y = 0;
        teleOp.gamepad1.right_stick_x = -0.5f;
        drive(nugget, teleOp, 0.5);
        teleOp.gamepad1.left_stick_y = -0.5f;
        teleOp.gamepad1.right_stick_x = 0.3f;
        drive(nugget, teleOp, 1);

        teleOp.localizer.loop();

        Pose2d truth = start.inverse().times(nugget.pose());
        Pose2d believed = teleOp.localizer.pose();
        assertTrue("it went somewhere: " + truth, truth.position.norm() > 12);
        assertEquals("x, having gone to " + truth, truth.position.x, believed.position.x, 0.25);
        assertEquals("y, having gone to " + truth, truth.position.y, believed.position.y, 0.25);
        assertEquals("heading, having gone to " + truth, 0, truth.heading.minus(believed.heading), 0.001);
    }

    @Test
    public void nuggetBelievesItHasTurnedAsFarAsTheSimulatorHasTurnedIt() {
        LocalizingTeleOp teleOp = started(nugget, new LocalizingTeleOp());
        nugget.setPose(ROOM_TO_TURN);
        teleOp.gamepad1.right_stick_x = -0.5f;

        drive(nugget, teleOp, 1);
        teleOp.localizer.loop();

        double turned = nugget.pose().heading.toDouble();
        assertTrue("it turned a good way: " + turned, turned > 1);
        assertEquals(turned, teleOp.localizer.pose().heading.toDouble(), 0.001);
    }

    @Test
    public void nuggetTurningAsItDrivesGripsTheFloorRatherThanSlidingOutOfTheTurn() {
        NuggetTeleOp teleOp = started(nugget, new NuggetTeleOp());
        nugget.setPose(new Pose2d(-48, -48, 0));
        teleOp.gamepad1.left_stick_y = -0.6f;
        teleOp.gamepad1.right_stick_x = -0.3f;

        for (int loop = 0; loop < 50; loop++) {
            Pose2d before = nugget.pose();
            teleOp.loop();
            nugget.step(SimRunner.LOOP_SECONDS);
            double aside = nugget.pose().minus(before).line.y;
            assertEquals("on loop " + loop + " it slid aside", 0, aside, 0.01);
        }
        assertTrue("and it turned: " + nugget.pose(), nugget.pose().heading.toDouble() > 0.5);
    }

    @Test
    public void nuggetHasNowhereToHoldABallSoItStartsWithNone() {
        assertEquals(0, nugget.held());
        assertEquals("where Reginald is preloaded", SimRobot.PRELOAD, new SimRobot().held());
    }

    @Test
    public void nuggetsPowersAreItsLeftMotorsThenItsRightsAsItsMotorsAreNamed() {
        NuggetTeleOp teleOp = started(nugget, new NuggetTeleOp());
        teleOp.gamepad1.right_stick_x = -0.4f;

        teleOp.loop();

        assertArrayEquals(new double[] {-0.4, 0.4}, nugget.powers(), 1e-6);
        assertEquals(TeamRobot.NUGGET.drivebase().motors().size(), nugget.powers().length);
    }

    @Test
    public void reginaldsPowersAreItsFourWheelsAsItsMotorsAreNamed() {
        SimRobot reginald = new SimRobot();
        reginald.leftFront.power = 0.1;
        reginald.rightFront.power = 0.2;
        reginald.leftBack.power = 0.3;
        reginald.rightBack.power = 0.4;

        assertArrayEquals(new double[] {0.1, 0.2, 0.3, 0.4}, reginald.powers(), EXACTLY);
        assertEquals(TeamRobot.REGINALD.drivebase().motors().size(), reginald.powers().length);
        assertEquals(TeamRobot.REGINALD, reginald.robot());
        assertEquals(TeamRobot.NUGGET, nugget.robot());
    }

    @Test
    public void anOpModeOfOneRobotIsNotWiredToTheOther() {
        IllegalArgumentException reginalds =
                assertThrows(IllegalArgumentException.class, () -> nugget.wire(new TestTeleOps.StickTeleOp()));
        IllegalArgumentException nuggets =
                assertThrows(IllegalArgumentException.class, () -> new SimRobot().wire(new NuggetTeleOp()));

        assertTrue(reginalds.getMessage(), reginalds.getMessage().contains("Nugget"));
        assertTrue(reginalds.getMessage(), reginalds.getMessage().contains(TestTeleOps.StickTeleOp.class.getName()));
        assertTrue(nuggets.getMessage(), nuggets.getMessage().contains("Reginald"));
        assertTrue(nuggets.getMessage(), nuggets.getMessage().contains(NuggetTeleOp.class.getName()));
    }
}
