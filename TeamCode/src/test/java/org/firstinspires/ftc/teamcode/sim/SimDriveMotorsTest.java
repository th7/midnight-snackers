package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimDriveMotorsTest {
    private static final double RUN_SECONDS = 1.5;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class DrivesStraightAhead extends OpMode {
        private List<DcMotorEx> motors;

        @Override
        public void init() {
            DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            motors = List.of(leftFront, rightFront, leftBack, rightBack);
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        @Override
        public void loop() {
            for (DcMotorEx motor : motors) {
                motor.setPower(0.5);
            }
        }
    }

    private Pose2d endOfARunOn(RobotDescription description) {
        SimRobot sim = new SimRobot(description, SimNoise.NONE);
        sim.setPose(new Pose2d(0, 0, 0));
        List<SimRecording.Tick> ticks = SimRunner.record(
                        new SimRecording("DrivesStraightAhead"),
                        new DrivesStraightAhead(),
                        sim,
                        RUN_SECONDS,
                        folder.getRoot().toPath(),
                        new SimDriverStation(),
                        SimRunner.Pace.FASTEST)
                .ticks();
        return ticks.get(ticks.size() - 1).truePose;
    }

    private static RobotDescription leftFrontReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.LEFT_FRONT, true));
    }

    @Test
    public void codeThatDrivesStraightAheadGoesStraightAheadOnTheDriveMotorsAsBuilt() {
        Pose2d end = endOfARunOn(RobotDescription.reginalds());

        assertTrue("drove ahead: " + end, end.position.x > 10);
        assertEquals(0, end.position.y, 1e-6);
        assertEquals(0, end.heading.toDouble(), 1e-6);
    }

    @Test
    public void theSameCodeVeersWithTheLeftFrontMotorReversedOnTheRobotTab() {
        Pose2d end = endOfARunOn(leftFrontReversed());

        assertTrue("turned left: " + end, end.heading.toDouble() > 0.2);
    }

    @Test
    public void aRunOnAReversedDriveMotorNamesIt() {
        assertEquals(
                "left_front drives backward on positive power (built drives forward)",
                SimDescription.described(leftFrontReversed()));
    }
}
