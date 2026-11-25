package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.framework.drive.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimLocalizationTest {
    private static final double DEAD_WHEEL_IN_PER_TICK = 0.0005433029;
    private static final double PAR_Y_TICKS_WHERE_THE_DESCRIPTION_PUTS_IT = 460.14847334700397;
    private static final double PERP_X_TICKS_WHERE_THE_DESCRIPTION_PUTS_IT = -6442.078626858056;
    private static final double DRIVE_SECONDS = 1.5;
    private static final double TURN_SECONDS = 1.5;
    private static final double RUN_SECONDS = DRIVE_SECONDS + TURN_SECONDS + 0.5;
    private static final double WITHIN_IN = 0.5;
    private static final double WITHIN_RADIANS = Math.toRadians(2);

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private final SimRobot sim = new SimRobot();

    private static final class DrivesAndTurns extends OpMode {
        private final SimRobot sim;
        private final RevHubOrientationOnRobot orientation;
        private TwoDeadWheelLocalizer localizer;
        private IMU imu;
        private DcMotorEx[] left;
        private DcMotorEx[] right;
        private long startedAt;
        private double fastestTurnSeen;
        private double yawLastRead;

        DrivesAndTurns(SimRobot sim, RevHubOrientationOnRobot orientation) {
            this.sim = sim;
            this.orientation = orientation;
        }

        @Override
        public void init() {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientation));
            DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            left = new DcMotorEx[] {leftFront, leftBack};
            right = new DcMotorEx[] {rightFront, rightBack};
            for (DcMotorEx wheel : new DcMotorEx[] {leftFront, leftBack, rightFront, rightBack}) {
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, DEAD_WHEEL_IN_PER_TICK, sim.pose(), sim::nanoTime);
            localizer.par.setDirection(DcMotorSimple.Direction.FORWARD);
            localizer.perp.setDirection(DcMotorSimple.Direction.FORWARD);
            localizer.update();
            startedAt = sim.nanoTime();
        }

        @Override
        public void loop() {
            PoseVelocity2d velocity = localizer.update();
            yawLastRead = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            fastestTurnSeen = Math.max(fastestTurnSeen, Math.abs(velocity.angVel));
            double seconds = (sim.nanoTime() - startedAt) / 1e9;
            if (seconds < DRIVE_SECONDS) {
                powers(0.5, 0.5);
            } else if (seconds < DRIVE_SECONDS + TURN_SECONDS) {
                powers(-0.4, 0.4);
            } else {
                powers(0, 0);
            }
        }

        private void powers(double leftPower, double rightPower) {
            for (DcMotorEx wheel : left) {
                wheel.setPower(leftPower);
            }
            for (DcMotorEx wheel : right) {
                wheel.setPower(rightPower);
            }
        }
    }

    private DrivesAndTurns run(LogoFacingDirection logo, UsbFacingDirection usb) {
        TwoDeadWheelLocalizer.Params code = TwoDeadWheelLocalizer.PARAMS;
        TwoDeadWheelLocalizer.Params matching = new TwoDeadWheelLocalizer.Params();
        matching.parYTicks = PAR_Y_TICKS_WHERE_THE_DESCRIPTION_PUTS_IT;
        matching.perpXTicks = PERP_X_TICKS_WHERE_THE_DESCRIPTION_PUTS_IT;
        TwoDeadWheelLocalizer.PARAMS = matching;
        try {
            sim.setPose(new Pose2d(-60, 0, 0));
            DrivesAndTurns opMode = new DrivesAndTurns(sim, new RevHubOrientationOnRobot(logo, usb));
            SimRunner.record(
                    new SimRecording("DrivesAndTurns"),
                    opMode,
                    sim,
                    RUN_SECONDS,
                    folder.getRoot().toPath(),
                    new SimDriverStation(),
                    SimRunner.Pace.FASTEST);
            return opMode;
        } finally {
            TwoDeadWheelLocalizer.PARAMS = code;
        }
    }

    @Test
    public void anOpModeWhoseHubAndDeadWheelsMatchTheDescriptionLocalizesThroughADriveAndATurn() {
        DrivesAndTurns opMode = run(LogoFacingDirection.RIGHT, UsbFacingDirection.UP);

        Pose2d believed = opMode.localizer.getPose();
        Pose2d truth = sim.pose();
        assertTrue("it drove a good way: " + truth, truth.position.x > -60 + 20);
        assertTrue("and turned a good way: " + truth, truth.heading.toDouble() > 1);
        assertEquals(truth.position.x, believed.position.x, WITHIN_IN);
        assertEquals(truth.position.y, believed.position.y, WITHIN_IN);
        assertEquals(truth.heading.toDouble(), believed.heading.toDouble(), WITHIN_RADIANS);
        assertEquals("the turn shows in the velocity it reads", 1, opMode.fastestTurnSeen, 0.9);
    }

    @Test
    public void theSameOpModeTakingTheHubForLogoDownUsbRightReadsItsYawHalfATurnOffAndNoTurnRate() {
        DrivesAndTurns opMode = run(LogoFacingDirection.DOWN, UsbFacingDirection.RIGHT);

        Pose2d truth = sim.pose();
        assertEquals(
                "yaw " + opMode.yawLastRead + " against " + truth.heading.toDouble(),
                Math.PI,
                Math.abs(Math.IEEEremainder(opMode.yawLastRead - truth.heading.toDouble(), 2 * Math.PI)),
                0.1);
        assertEquals("no turn shows in the velocity it reads", 0, opMode.fastestTurnSeen, 1e-6);
    }
}
