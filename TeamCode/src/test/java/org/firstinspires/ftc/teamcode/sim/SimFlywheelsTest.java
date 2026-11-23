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
import org.firstinspires.ftc.teamcode.simcore.Flywheels;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimFlywheelsTest {
    private static final double RUN_SECONDS = 8;
    private static final double TICKS_PER_SECOND = 1200;
    private static final double RPM_AT_TARGET = 1200.0 / 28 * 60;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class SpinsTheFlywheels extends OpMode {
        private final DcMotorSimple.Direction front;
        private final double frontTicksPerSecond;
        private final double backTicksPerSecond;
        private DcMotorEx flywheelFront;
        private DcMotorEx flywheelBack;
        double frontMeasured;
        double backMeasured;

        SpinsTheFlywheels(DcMotorSimple.Direction front, double frontTicksPerSecond, double backTicksPerSecond) {
            this.front = front;
            this.frontTicksPerSecond = frontTicksPerSecond;
            this.backTicksPerSecond = backTicksPerSecond;
        }

        @Override
        public void init() {
            flywheelFront = hardwareMap.get(DcMotorEx.class, "flywheelFront");
            flywheelBack = hardwareMap.get(DcMotorEx.class, "flywheelBack");
            flywheelFront.setDirection(front);
            flywheelBack.setDirection(DcMotorSimple.Direction.FORWARD);
            for (DcMotorEx flywheel : List.of(flywheelFront, flywheelBack)) {
                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        @Override
        public void loop() {
            flywheelFront.setVelocity(frontTicksPerSecond);
            flywheelBack.setVelocity(backTicksPerSecond);
            frontMeasured = flywheelFront.getVelocity();
            backMeasured = flywheelBack.getVelocity();
        }
    }

    private List<SimRecording.Tick> run(RobotDescription description, SpinsTheFlywheels opMode) {
        SimRobot sim = new SimRobot(description, SimNoise.NONE);
        sim.setPose(new Pose2d(0, 0, 0));
        return SimRunner.record(
                        new SimRecording("SpinsTheFlywheels"),
                        opMode,
                        sim,
                        RUN_SECONDS,
                        folder.getRoot().toPath(),
                        new SimDriverStation(),
                        SimRunner.Pace.FASTEST)
                .ticks();
    }

    private static SimRecording.Mechanisms last(List<SimRecording.Tick> ticks) {
        return ticks.get(ticks.size() - 1).mechanisms;
    }

    private static SimRecording.Mechanisms at(List<SimRecording.Tick> ticks, double seconds) {
        for (SimRecording.Tick tick : ticks) {
            if (tick.seconds >= seconds) {
                return tick.mechanisms;
            }
        }
        throw new AssertionError("no tick at " + seconds + " s");
    }

    @Test
    public void bothWheelsSetTheRightWayComeUpToTheVelocityTheCodeAsksForAndLaunch() {
        SpinsTheFlywheels opMode =
                new SpinsTheFlywheels(DcMotorSimple.Direction.REVERSE, TICKS_PER_SECOND, TICKS_PER_SECOND);
        SimRecording.Mechanisms last = last(run(RobotDescription.reginalds(), opMode));

        assertEquals(Flywheels.Way.LAUNCHING, last.front().way());
        assertEquals(Flywheels.Way.LAUNCHING, last.back().way());
        assertEquals(RPM_AT_TARGET, last.front().rpm(), RPM_AT_TARGET * 0.02);
        assertEquals(RPM_AT_TARGET, last.back().rpm(), RPM_AT_TARGET * 0.02);
        assertEquals(TICKS_PER_SECOND, opMode.frontMeasured, TICKS_PER_SECOND * 0.02);
        assertEquals(TICKS_PER_SECOND, opMode.backMeasured, TICKS_PER_SECOND * 0.02);
    }

    @Test
    public void eachWheelTakesTimeToComeUpToSpeed() {
        List<SimRecording.Tick> ticks = run(
                RobotDescription.reginalds(),
                new SpinsTheFlywheels(DcMotorSimple.Direction.REVERSE, TICKS_PER_SECOND, TICKS_PER_SECOND));

        double early = at(ticks, 0.2).front().rpm();
        assertTrue("turning the launch way at 0.2 s: " + early, early > 0);
        assertTrue("short of its speed at 0.2 s: " + early, early < RPM_AT_TARGET * 0.5);
    }

    @Test
    public void theFrontWheelLeftForwardTurnsTheWrongWayAndTheRunGoesOn() {
        List<SimRecording.Tick> ticks = run(
                RobotDescription.reginalds(),
                new SpinsTheFlywheels(DcMotorSimple.Direction.FORWARD, TICKS_PER_SECOND, TICKS_PER_SECOND));

        assertEquals(Flywheels.Way.WRONG_WAY, last(ticks).front().way());
        assertEquals(Flywheels.Way.LAUNCHING, last(ticks).back().way());
        assertEquals(RUN_SECONDS, ticks.get(ticks.size() - 1).seconds, 0.1);
    }

    @Test
    public void aFrontFlywheelReversedOnTheRobotTabLaunchesLeftForward() {
        RobotDescription reversed =
                RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.FLYWHEEL_FRONT, true));

        SimRecording.Mechanisms last = last(run(
                reversed, new SpinsTheFlywheels(DcMotorSimple.Direction.FORWARD, TICKS_PER_SECOND, TICKS_PER_SECOND)));

        assertEquals(Flywheels.Way.LAUNCHING, last.front().way());
    }

    @Test
    public void aFasterFrontWheelShowsHowMuchFasterItsSurfaceMoves() {
        SimRecording.Mechanisms last = last(run(
                RobotDescription.reginalds(),
                new SpinsTheFlywheels(DcMotorSimple.Direction.REVERSE, TICKS_PER_SECOND, 0.8 * TICKS_PER_SECOND)));

        double surfaceAtTarget = Math.PI * 3.78 * RPM_AT_TARGET / 60;
        assertEquals(0.2 * surfaceAtTarget, last.mismatch(), 0.04 * surfaceAtTarget);
    }

    @Test
    public void aWheelNobodyDrivesIsStill() {
        SimRecording.Mechanisms last =
                last(run(RobotDescription.reginalds(), new SpinsTheFlywheels(DcMotorSimple.Direction.REVERSE, 0, 0)));

        assertEquals(Flywheels.Turning.still().way(), last.front().way());
        assertEquals(0, last.front().rpm(), 0);
        assertEquals(0, last.mismatch(), 0);
    }
}
