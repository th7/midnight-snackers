package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Handover;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimHandoverTest {
    private static final double TAKE_IN_SECONDS = 2.0;
    private static final double RUN_SECONDS = TAKE_IN_SECONDS + 1.5;
    private static final double LANE_Y = -40;
    private static final double[] POLLEN_X = {8, 14};
    private static final double HALF_TURN_AT_NINETY_PERCENT_SECONDS = 0.5 / (0.9 * 59.5 / 60);
    private static final double ONE_LOOP_SECONDS = 0.05;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class TakesInThenSpins extends OpMode {
        private final SimRobot sim;
        private final double spinnerPower;
        private final double rampPosition;
        private final List<DcMotorEx> wheels = new ArrayList<>();
        private DcMotorEx intake;
        private CRServo spinner;
        private Servo ramp;
        private long startedAt;

        TakesInThenSpins(SimRobot sim, double spinnerPower, double rampPosition) {
            this.sim = sim;
            this.spinnerPower = spinnerPower;
            this.rampPosition = rampPosition;
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
            spinner = hardwareMap.get(CRServo.class, "spinner");
            ramp = hardwareMap.get(Servo.class, "ramp");
            startedAt = sim.nanoTime();
        }

        @Override
        public void loop() {
            boolean takingIn = (sim.nanoTime() - startedAt) / 1e9 < TAKE_IN_SECONDS;
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(takingIn ? 0.3 : 0);
            }
            intake.setPower(1);
            ramp.setPosition(rampPosition);
            spinner.setPower(takingIn ? 0 : spinnerPower);
        }
    }

    private static final class Run {
        final SimRobot sim;
        final List<SimRobot.Piece> pollen;
        final List<SimRecording.Tick> ticks;

        Run(SimRobot sim, List<SimRobot.Piece> pollen, List<SimRecording.Tick> ticks) {
            this.sim = sim;
            this.pollen = pollen;
            this.ticks = ticks;
        }

        SimRecording.Tick last() {
            return ticks.get(ticks.size() - 1);
        }

        List<Double> fedAt() {
            List<Double> at = new ArrayList<>();
            int fed = 0;
            for (SimRecording.Tick tick : ticks) {
                for (; fed < tick.mechanisms.fed(); fed++) {
                    at.add(tick.seconds);
                }
            }
            return at;
        }
    }

    private Run run(double spinnerPower, double rampPosition) {
        SimRobot sim = new SimRobot(RobotDescription.reginalds(), SimNoise.NONE);
        List<SimRobot.Piece> pollen = new ArrayList<>();
        for (SimRobot.Piece ball : sim.balls()) {
            if (ball.kind() == Field.Kind.POLLEN
                    && ball.setUpFrom().place() instanceof Field.Place.Loose
                    && pollen.size() < POLLEN_X.length) {
                pollen.add(ball);
            }
        }
        for (int i = 0; i < pollen.size(); i++) {
            sim.place(pollen.get(i), POLLEN_X[i], LANE_Y);
        }
        sim.setPose(new Pose2d(-8, LANE_Y, 0));
        List<SimRecording.Tick> ticks = SimRunner.record(
                        new SimRecording("TakesInThenSpins"),
                        new TakesInThenSpins(sim, spinnerPower, rampPosition),
                        sim,
                        RUN_SECONDS,
                        folder.getRoot().toPath(),
                        new SimDriverStation(),
                        SimRunner.Pace.FASTEST)
                .ticks();
        return new Run(sim, pollen, ticks);
    }

    private static Handover.Why whyWhileSpinning(Run run) {
        return run.last().mechanisms.handing();
    }

    @Test
    public void spinningClockwiseWithTheRampUpFeedsBothHeldPollenHalfATurnApart() {
        Run run = run(1, 0.15);

        List<Double> fedAt = run.fedAt();
        assertEquals("fed at " + fedAt, 2, fedAt.size());
        assertTrue("both were held before the spinner turned: " + fedAt, fedAt.get(0) > TAKE_IN_SECONDS);
        assertEquals(HALF_TURN_AT_NINETY_PERCENT_SECONDS, fedAt.get(1) - fedAt.get(0), ONE_LOOP_SECONDS);
        assertEquals(0, run.last().mechanisms.held());
        assertEquals(Handover.Why.NOTHING_HELD, whyWhileSpinning(run));
        int last = run.ticks.size() - 1;
        for (SimRobot.Piece fed : run.pollen) {
            assertNull(
                    "a fed pollen has left the robot",
                    run.ticks.get(last).pieces[run.sim.balls().indexOf(fed)]);
        }
    }

    @Test
    public void spinningCounterclockwiseFeedsNoneAndEachTickSaysWhy() {
        Run run = run(-1, 0.15);

        assertEquals(List.of(), run.fedAt());
        assertEquals(2, run.last().mechanisms.held());
        assertEquals(Handover.Why.SPINNER_NOT_CLOCKWISE, whyWhileSpinning(run));
    }

    @Test
    public void withTheRampDownItFeedsNoneAndEachTickSaysWhy() {
        Run run = run(1, 0.1);

        assertEquals(List.of(), run.fedAt());
        assertEquals(2, run.last().mechanisms.held());
        assertEquals(Handover.Why.RAMP_DOWN, whyWhileSpinning(run));
    }
}
