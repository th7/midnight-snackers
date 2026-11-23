package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Spinner;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimSpindexerTest {
    private static final double RUN_SECONDS = 0.5;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class SpinsAndRaises extends OpMode {
        private final double spinnerPower;
        private final DcMotorSimple.Direction direction;
        private final PwmControl.PwmRange range;
        private final double rampPosition;
        private CRServo spinner;
        private Servo ramp;

        SpinsAndRaises(
                double spinnerPower,
                DcMotorSimple.Direction direction,
                PwmControl.PwmRange range,
                double rampPosition) {
            this.spinnerPower = spinnerPower;
            this.direction = direction;
            this.range = range;
            this.rampPosition = rampPosition;
        }

        @Override
        public void init() {
            spinner = hardwareMap.get(CRServo.class, "spinner");
            spinner.setDirection(direction);
            if (range != null) {
                ((PwmControl) spinner).setPwmRange(range);
            }
            ramp = hardwareMap.get(Servo.class, "ramp");
        }

        @Override
        public void loop() {
            spinner.setPower(spinnerPower);
            ramp.setPosition(rampPosition);
        }
    }

    private static final class Pulses extends OpMode {
        private final double spinnerUs;
        private final double rampUs;
        private PwmControl spinner;
        private PwmControl ramp;

        Pulses(double spinnerUs, double rampUs) {
            this.spinnerUs = spinnerUs;
            this.rampUs = rampUs;
        }

        @Override
        public void init() {
            spinner = (PwmControl) hardwareMap.get(CRServo.class, "spinner");
            ramp = (PwmControl) hardwareMap.get(Servo.class, "ramp");
        }

        @Override
        public void loop() {
            spinner.setPulseWidth(spinnerUs);
            ramp.setPulseWidth(rampUs);
        }
    }

    private static final class ReadsPulsesBack extends OpMode {
        private final List<Double> spinnerReads = new ArrayList<>();
        private final List<Double> rampReads = new ArrayList<>();
        private CRServo spinner;
        private Servo ramp;

        @Override
        public void init() {
            spinner = hardwareMap.get(CRServo.class, "spinner");
            ramp = hardwareMap.get(Servo.class, "ramp");
        }

        @Override
        public void start() {
            ((PwmControl) spinner).setPulseWidth(1234.7);
            spinnerReads.add(((PwmControl) spinner).getPulseWidth());
            spinner.setPower(1);
            spinnerReads.add(((PwmControl) spinner).getPulseWidth());
            ((PwmControl) ramp).setPulseWidth(1000);
            rampReads.add(((PwmControl) ramp).getPulseWidth());
            ramp.setPosition(0.15);
            rampReads.add(((PwmControl) ramp).getPulseWidth());
        }

        @Override
        public void loop() {}
    }

    private List<SimRecording.Tick> run(RobotDescription description, OpMode opMode) {
        SimRobot sim = new SimRobot(description, SimNoise.NONE);
        sim.setPose(new Pose2d(0, 0, 0));
        return SimRunner.record(
                        new SimRecording(opMode.getClass().getSimpleName()),
                        opMode,
                        sim,
                        RUN_SECONDS,
                        folder.getRoot().toPath(),
                        new SimDriverStation(),
                        SimRunner.Pace.FASTEST)
                .ticks();
    }

    private List<SimRecording.Tick> spun(double power, DcMotorSimple.Direction direction, PwmControl.PwmRange range) {
        return run(RobotDescription.reginalds(), new SpinsAndRaises(power, direction, range, 0));
    }

    private static SimRecording.Mechanisms last(List<SimRecording.Tick> ticks) {
        return ticks.get(ticks.size() - 1).mechanisms;
    }

    private static double firstUpAfter(List<SimRecording.Tick> ticks) {
        double first = ticks.get(0).seconds;
        for (SimRecording.Tick tick : ticks) {
            if (tick.mechanisms.rampUp()) {
                return tick.seconds - first;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    @Test
    public void theSpinnerAtPlusOneForwardTurnsClockwiseAtNinetyPercent() {
        Spinner.Turning turning =
                last(spun(1, DcMotorSimple.Direction.FORWARD, null)).spinner();

        assertEquals(Spinner.Way.CLOCKWISE, turning.way());
        assertEquals(0.9, turning.ofFull(), 1e-9);
    }

    @Test
    public void theSpinnerAtMinusOneOrReversedTurnsCounterclockwise() {
        assertEquals(
                Spinner.Way.COUNTERCLOCKWISE,
                last(spun(-1, DcMotorSimple.Direction.FORWARD, null)).spinner().way());
        assertEquals(
                Spinner.Way.COUNTERCLOCKWISE,
                last(spun(1, DcMotorSimple.Direction.REVERSE, null)).spinner().way());
    }

    @Test
    public void aPwmRangeOfFiveHundredToTwentyFiveHundredGivesTheSpinnerFullSpeed() {
        Spinner.Turning turning = last(spun(1, DcMotorSimple.Direction.FORWARD, new PwmControl.PwmRange(500, 2500)))
                .spinner();

        assertEquals(1, turning.ofFull(), 1e-9);
    }

    @Test
    public void aSpinnerTheDescriptionReversesTurnsTheOtherWay() {
        RobotDescription reversed =
                RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.SPINNER, true));

        Spinner.Turning turning = last(run(reversed, new SpinsAndRaises(1, DcMotorSimple.Direction.FORWARD, null, 0)))
                .spinner();

        assertEquals(Spinner.Way.COUNTERCLOCKWISE, turning.way());
    }

    @Test
    public void aSpinnerNobodyHasDrivenIsStill() {
        List<SimRecording.Tick> ticks = spun(0.7, DcMotorSimple.Direction.FORWARD, null);

        assertEquals(Spinner.Turning.still(), ticks.get(0).mechanisms.spinner());
    }

    @Test
    public void theRampAtPointOneFiveIsUpAfterSlewingAboutATenthOfASecond() {
        List<SimRecording.Tick> ticks =
                run(RobotDescription.reginalds(), new SpinsAndRaises(0, DcMotorSimple.Direction.FORWARD, null, 0.15));

        double upAfter = firstUpAfter(ticks);
        assertTrue("up after " + upAfter + " s", upAfter >= 0.1 && upAfter < 0.15);
        assertEquals(36.45, last(ticks).rampDegrees(), 1e-9);
    }

    @Test
    public void theRampAtPointOneIsDown() {
        List<SimRecording.Tick> ticks =
                run(RobotDescription.reginalds(), new SpinsAndRaises(0, DcMotorSimple.Direction.FORWARD, null, 0.1));

        assertFalse(last(ticks).rampUp());
        assertEquals(24.3, last(ticks).rampDegrees(), 1e-9);
    }

    @Test
    public void aSpinnerPulsedAtTwentyFourHundredMicrosecondsTurnsClockwiseAtNinetyPercent() {
        Spinner.Turning turning =
                last(run(RobotDescription.reginalds(), new Pulses(2400, 600))).spinner();

        assertEquals(Spinner.Way.CLOCKWISE, turning.way());
        assertEquals(0.9, turning.ofFull(), 1e-9);
    }

    @Test
    public void aSpinnerPulsedAtSixHundredMicrosecondsTurnsCounterclockwiseAtNinetyPercent() {
        Spinner.Turning turning =
                last(run(RobotDescription.reginalds(), new Pulses(600, 600))).spinner();

        assertEquals(Spinner.Way.COUNTERCLOCKWISE, turning.way());
        assertEquals(0.9, turning.ofFull(), 1e-9);
    }

    @Test
    public void aRampPulsedAtEightHundredSeventyMicrosecondsIsUpAfterItsSlew() {
        List<SimRecording.Tick> ticks = run(RobotDescription.reginalds(), new Pulses(1500, 870));

        double upAfter = firstUpAfter(ticks);
        assertTrue("up after " + upAfter + " s", upAfter >= 0.1 && upAfter < 0.15);
        assertEquals(36.45, last(ticks).rampDegrees(), 1e-9);
    }

    @Test
    public void aRampPulsedAtEightHundredFiftyMicrosecondsIsDown() {
        List<SimRecording.Tick> ticks = run(RobotDescription.reginalds(), new Pulses(1500, 850));

        assertFalse(last(ticks).rampUp());
        assertEquals(33.75, last(ticks).rampDegrees(), 1e-9);
    }

    @Test
    public void aPulseWidthReadsBackWhatWasSetAndThenWhatALaterPowerOrPositionSent() {
        ReadsPulsesBack opMode = new ReadsPulsesBack();

        run(RobotDescription.reginalds(), opMode);

        assertEquals(List.of(1234.0, 2400.0), opMode.spinnerReads);
        assertEquals(List.of(1000.0, 870.0), opMode.rampReads);
    }
}
