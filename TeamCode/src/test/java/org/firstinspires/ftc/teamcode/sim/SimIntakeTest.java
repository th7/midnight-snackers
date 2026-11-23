package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Intake;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimIntakeTest {
    private static final double RUN_SECONDS = 2.0;
    private static final double DRIVE_POWER = 0.3;
    private static final double LANE_Y = -40;
    private static final double[] POLLEN_X = {8, 14, 20};

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class DrivesOverPollen extends OpMode {
        private final double intakePower;
        private final List<DcMotorEx> wheels = new ArrayList<>();
        private DcMotorEx intake;

        DrivesOverPollen(double intakePower) {
            this.intakePower = intakePower;
        }

        @Override
        public void init() {
            for (String name : new String[] {"leftFront", "leftBack", "rightFront", "rightBack"}) {
                DcMotorEx wheel = hardwareMap.get(DcMotorEx.class, name);
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wheels.add(wheel);
            }
            hardwareMap.get(DcMotorEx.class, "leftBack").setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareMap.get(DcMotorEx.class, "rightBack").setDirection(DcMotorSimple.Direction.REVERSE);
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        }

        @Override
        public void loop() {
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(DRIVE_POWER);
            }
            intake.setPower(intakePower);
        }
    }

    private static final class Run {
        final SimRobot sim;
        final List<SimRobot.Piece> pollen;
        final SimRecording recording;

        Run(SimRobot sim, List<SimRobot.Piece> pollen, SimRecording recording) {
            this.sim = sim;
            this.pollen = pollen;
            this.recording = recording;
        }

        SimRecording.Tick last() {
            List<SimRecording.Tick> ticks = recording.ticks();
            return ticks.get(ticks.size() - 1);
        }

        double[] placeInLastTick(int which) {
            return last().pieces[sim.balls().indexOf(pollen.get(which))];
        }
    }

    private Run run(RobotDescription description, int pollenAhead, double intakePower) {
        SimRobot sim = new SimRobot(description, SimNoise.NONE);
        List<SimRobot.Piece> pollen = new ArrayList<>();
        for (SimRobot.Piece ball : sim.balls()) {
            if (ball.kind() == Field.Kind.POLLEN
                    && ball.setUpFrom().place() instanceof Field.Place.Loose
                    && pollen.size() < pollenAhead) {
                pollen.add(ball);
            }
        }
        for (int i = 0; i < pollen.size(); i++) {
            sim.place(pollen.get(i), POLLEN_X[i], LANE_Y);
        }
        sim.setPose(new Pose2d(-8, LANE_Y, 0));
        SimRecording recording = SimRunner.record(
                new SimRecording("DrivesOverPollen"),
                new DrivesOverPollen(intakePower),
                sim,
                RUN_SECONDS,
                folder.getRoot().toPath(),
                new SimDriverStation(),
                SimRunner.Pace.FASTEST);
        return new Run(sim, pollen, recording);
    }

    private static RobotDescription intakeReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.INTAKE, true));
    }

    private static void assertPushedAhead(Run run, int which) {
        double[] ball = run.placeInLastTick(which);
        assertNotNull("pollen " + which + " is still on the field", ball);
        double front = run.last().truePose.position.x + SimPlacement.ROBOT_SIZE_IN / 2;
        assertTrue("pollen " + which + " is ahead of the front edge: " + ball[0] + " vs " + front, ball[0] > front);
    }

    @Test
    public void intakePowerOneOverAPollenTakesItIn() {
        Run run = run(RobotDescription.reginalds(), 1, 1);

        assertTrue("the robot drove past where the pollen lay", run.last().truePose.position.x > 8);
        assertNull("the pollen is in the robot, not on the field", run.placeInLastTick(0));
        assertEquals(1, run.last().mechanisms.held());
        assertEquals(Intake.Way.IN, run.last().mechanisms.intake());
    }

    @Test
    public void intakePowerMinusOneOverAPollenPushesIt() {
        Run run = run(RobotDescription.reginalds(), 1, -1);

        assertPushedAhead(run, 0);
        assertEquals(0, run.last().mechanisms.held());
        assertEquals(Intake.Way.OUT, run.last().mechanisms.intake());
    }

    @Test
    public void aThirdPollenWithTwoHeldIsPushed() {
        Run run = run(RobotDescription.reginalds(), 3, 1);

        assertNull(run.placeInLastTick(0));
        assertNull(run.placeInLastTick(1));
        assertPushedAhead(run, 2);
        assertEquals(2, run.last().mechanisms.held());
    }

    @Test
    public void reversingTheIntakeMakesPowerOnePushAndMinusOneTakeIn() {
        Run pushes = run(intakeReversed(), 1, 1);
        Run takes = run(intakeReversed(), 1, -1);

        assertPushedAhead(pushes, 0);
        assertEquals(Intake.Way.OUT, pushes.last().mechanisms.intake());
        assertNull(takes.placeInLastTick(0));
        assertEquals(Intake.Way.IN, takes.last().mechanisms.intake());
        assertEquals(1, takes.last().mechanisms.held());
    }

    @Test
    public void anIntakeLeftStillShowsStillAndHoldsNothing() {
        Run run = run(RobotDescription.reginalds(), 1, 0);

        assertPushedAhead(run, 0);
        assertEquals(Intake.Way.STILL, run.last().mechanisms.intake());
        assertEquals(0, run.last().mechanisms.held());
    }
}
