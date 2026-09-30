package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.nugget.NuggetOpMode;
import org.firstinspires.ftc.nugget.NuggetTeleOp;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Test;

public class SimNuggetTest {
    private static final Pose2d CLEAR_OF_EVERYTHING = new Pose2d(-60, 0, 0);

    private static final Pose2d ROOM_TO_TURN = new Pose2d(-40, 0, 0);

    private static final double EXACTLY = 1e-9;

    private final SimRobot nugget = new SimRobot(TeamRobot.NUGGET, SimNoise.NONE);

    public static final class NeitherReversed extends NuggetOpMode {
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
        SimRobot seeded = new SimRobot(TeamRobot.NUGGET, Noise.seeded(7));
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
