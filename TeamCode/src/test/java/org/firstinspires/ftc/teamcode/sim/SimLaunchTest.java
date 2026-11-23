package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
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
import org.firstinspires.ftc.teamcode.simcore.Launch;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.firstinspires.ftc.teamcode.simcore.WheelPair;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimLaunchTest {
    private static final double TAKE_IN_SECONDS = 2.0;
    private static final double UP_TO_SPEED_SECONDS = 8.0;
    private static final double LANE_Y = -40;
    private static final double[] POLLEN_X = {8, 14};
    private static final double TICKS_PER_SECOND = 1200;
    private static final double MOUNTED_PITCH_DEGREES = 70;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private record Wheels(DcMotorSimple.Direction front, double frontTicksPerSecond, double backTicksPerSecond) {
        static Wheels same() {
            return new Wheels(DcMotorSimple.Direction.REVERSE, TICKS_PER_SECOND, TICKS_PER_SECOND);
        }
    }

    private static final class TakesInThenLaunches extends OpMode {
        private final SimRobot sim;
        private final Wheels asked;
        private final double feedFromSeconds;
        private final List<DcMotorEx> drive = new ArrayList<>();
        private DcMotorEx intake;
        private DcMotorEx flywheelFront;
        private DcMotorEx flywheelBack;
        private CRServo spinner;
        private Servo ramp;
        private long startedAt;

        TakesInThenLaunches(SimRobot sim, Wheels asked, double feedFromSeconds) {
            this.sim = sim;
            this.asked = asked;
            this.feedFromSeconds = feedFromSeconds;
        }

        @Override
        public void init() {
            for (String name : new String[] {"leftFront", "leftBack", "rightFront", "rightBack"}) {
                DcMotorEx wheel = hardwareMap.get(DcMotorEx.class, name);
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.add(wheel);
            }
            hardwareMap.get(DcMotorEx.class, "leftBack").setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareMap.get(DcMotorEx.class, "rightBack").setDirection(DcMotorSimple.Direction.REVERSE);
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            flywheelFront = hardwareMap.get(DcMotorEx.class, "flywheelFront");
            flywheelBack = hardwareMap.get(DcMotorEx.class, "flywheelBack");
            flywheelFront.setDirection(asked.front());
            flywheelBack.setDirection(DcMotorSimple.Direction.FORWARD);
            for (DcMotorEx flywheel : List.of(flywheelFront, flywheelBack)) {
                flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            spinner = hardwareMap.get(CRServo.class, "spinner");
            ramp = hardwareMap.get(Servo.class, "ramp");
            startedAt = sim.nanoTime();
        }

        @Override
        public void loop() {
            double seconds = (sim.nanoTime() - startedAt) / 1e9;
            boolean takingIn = seconds < TAKE_IN_SECONDS;
            for (DcMotorEx wheel : drive) {
                wheel.setPower(takingIn ? 0.3 : 0);
            }
            intake.setPower(1);
            ramp.setPosition(0.15);
            boolean launching = !takingIn;
            flywheelFront.setVelocity(launching ? asked.frontTicksPerSecond() : 0);
            flywheelBack.setVelocity(launching ? asked.backTicksPerSecond() : 0);
            spinner.setPower(seconds >= feedFromSeconds ? 1 : 0);
        }
    }

    private static final class Run {
        final SimRobot sim;
        final List<SimRobot.Piece> pollen;
        final List<SimRecording.Tick> ticks;
        final double seconds;

        Run(SimRobot sim, List<SimRobot.Piece> pollen, List<SimRecording.Tick> ticks, double seconds) {
            this.sim = sim;
            this.pollen = pollen;
            this.ticks = ticks;
            this.seconds = seconds;
        }

        SimRecording.Mechanisms last() {
            return ticks.get(ticks.size() - 1).mechanisms;
        }

        WheelPair.Launched first() {
            List<WheelPair.Launched> launches = sim.launches();
            assertTrue("it launched: " + last(), !launches.isEmpty());
            return launches.get(0);
        }
    }

    private Run run(RobotDescription description, Wheels wheels, double feedFromSeconds, double runSeconds) {
        SimRobot sim = new SimRobot(description, SimNoise.NONE);
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
                        new SimRecording("TakesInThenLaunches"),
                        new TakesInThenLaunches(sim, wheels, feedFromSeconds),
                        sim,
                        runSeconds,
                        folder.getRoot().toPath(),
                        new SimDriverStation(),
                        SimRunner.Pace.FASTEST)
                .ticks();
        return new Run(sim, pollen, ticks, runSeconds);
    }

    private Run fedOnceUpToSpeed(Wheels wheels) {
        double feedFrom = TAKE_IN_SECONDS + UP_TO_SPEED_SECONDS;
        return run(RobotDescription.reginalds(), wheels, feedFrom, feedFrom + 1.5);
    }

    private static double pitchDegrees(Vec3 velocity) {
        return Math.toDegrees(Math.atan2(velocity.z(), Math.hypot(velocity.x(), velocity.y())));
    }

    private static double speed(Vec3 velocity) {
        return Math.sqrt(velocity.dot(velocity));
    }

    @Test
    public void bothWheelsAtTheSameVelocityLaunchEachFedPollenWithNoSpinAtTheMountingsAngleIntoAFlight() {
        Run run = fedOnceUpToSpeed(Wheels.same());

        assertEquals(2, run.last().launched());
        assertEquals(new SimRecording.Chamber.Empty(), run.last().chamber());
        WheelPair.Launched first = run.first();
        assertEquals(WheelPair.SpinKind.NONE, first.kind());
        assertEquals(MOUNTED_PITCH_DEGREES, pitchDegrees(first.velocity()), 0.5);
        double highest = 0;
        for (SimRecording.Tick tick : run.ticks) {
            double[] at = tick.pieces[run.sim.balls().indexOf(run.pollen.get(0))];
            if (at != null) {
                highest = Math.max(highest, at[2]);
            }
        }
        assertTrue("it flew up from the 14 in exit, as high as " + highest + " in", highest > 40);
    }

    @Test
    public void aFasterFrontWheelGivesBackspinAndAFasterBackWheelTopspin() {
        Run frontFaster =
                fedOnceUpToSpeed(new Wheels(DcMotorSimple.Direction.REVERSE, TICKS_PER_SECOND, 0.8 * TICKS_PER_SECOND));
        Run backFaster =
                fedOnceUpToSpeed(new Wheels(DcMotorSimple.Direction.REVERSE, 0.8 * TICKS_PER_SECOND, TICKS_PER_SECOND));

        assertEquals(WheelPair.SpinKind.BACKSPIN, frontFaster.first().kind());
        assertEquals(WheelPair.SpinKind.TOPSPIN, backFaster.first().kind());
    }

    @Test
    public void theFrontWheelLeftForwardTurnsTheWrongWayAndThePollenStaysInTheLauncher() {
        Run run = fedOnceUpToSpeed(new Wheels(DcMotorSimple.Direction.FORWARD, TICKS_PER_SECOND, TICKS_PER_SECOND));

        assertEquals(0, run.last().launched());
        assertEquals(List.of(), run.sim.launches());
        assertEquals(
                new SimRecording.Chamber.Holds(Launch.Why.FRONT_WHEEL_WRONG_WAY),
                run.last().chamber());
        assertEquals(1, run.last().fed());
        assertEquals(1, run.last().held());
        assertEquals(Handover.Why.LAUNCHER_FULL, run.last().handing());
        assertEquals(run.seconds, run.ticks.get(run.ticks.size() - 1).seconds, 0.1);
    }

    @Test
    public void feedingBeforeTheWheelsAreUpToSpeedLaunchesSlowerThanFeedingAfter() {
        Run early = run(RobotDescription.reginalds(), Wheels.same(), TAKE_IN_SECONDS, TAKE_IN_SECONDS + 1.5);
        Run late = fedOnceUpToSpeed(Wheels.same());

        double earlySpeed = speed(early.first().velocity());
        double lateSpeed = speed(late.first().velocity());
        assertTrue("early " + earlySpeed + " in/s, late " + lateSpeed + " in/s", earlySpeed < 0.9 * lateSpeed);
    }

    @Test
    public void aRunThatLaunchesStaysWithinTheWorkAFlightIsAllowed() {
        Run run = fedOnceUpToSpeed(Wheels.same());

        assertTrue(
                "the flight did " + run.sim.flightWork() + " units of work in " + run.seconds + " s",
                run.sim.flightWork() <= Shot.MOST_WORK_PER_SECOND * run.seconds);
    }
}
