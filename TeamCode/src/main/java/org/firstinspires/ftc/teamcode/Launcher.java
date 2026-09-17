package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Launcher extends SubSystem {
    /**
     * How long a launch holds the bottom gate open, in seconds: long enough for the chambered ball
     * to drop into the flywheel. The driver tunes it from gamepad 2 while the robot is on the
     * field, so it is one number the launch plan reads rather than a literal in a step.
     */
    public static final double BOTTOM_GATE_WAIT_SECONDS = 0.15;
    /** What one press of the driver's bumper moves {@link #BOTTOM_GATE_WAIT_SECONDS} by. */
    public static final double BOTTOM_GATE_WAIT_STEP_SECONDS = 0.01;

    private final LongSupplier clock;

    private final double topGateOpenPosition = 1;
    private final double topGateClosedPosition = 0.6;
    private final double bottomGateOpenPosition = 0.5;
    private final double bottomGateClosedPosition = 0.4;
    private final double closeLauncherPower = 1050d;
    private final DcMotorEx launcher;
    private final Servo topGate;
    private final Servo bottomGate; // bottomGate is closer to launcher
    private double topGatePosition = topGateOpenPosition;
    private double bottomGatePosition = bottomGateClosedPosition;
    private double launcherVelocity = 0d;
    private double bottomGateWaitSeconds = BOTTOM_GATE_WAIT_SECONDS;
    private final PlanRunner planRunner = new PlanRunner();

    public Launcher(DcMotorEx launcher, Servo topGate, Servo bottomGate, LongSupplier clock) {
        this.clock = clock;
        this.launcher = launcher;
        this.topGate = topGate;
        this.bottomGate = bottomGate;
    }

    @Override
    protected void onInit() {
        launcher.setPositionPIDFCoefficients(5);

        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);
    }

    @Override
    protected void onLoop() {
        planRunner.loop();
        launcher.setVelocity(launcherVelocity);
        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);
    }

    public void launchyLaunch() {
        if (planRunner.done()) {
            planRunner.run(launchPlan());
        }
    }

    private Plan launchPlan() {
        return new Plan(
                ensureFlywheelReady(),
                launchCloseTopGate(),
                launchOpenBottomGate(),
                Step.waitFor("ball to fall into launcher", bottomGateWaitSeconds, clock),
                launchCloseBottomGate(),
                launchOpenTopGate(),
                Step.waitFor("ball to fall into bottom position", 0.15, clock));
    }

    private Step ensureFlywheelReady() {
        return new Step(
                "ensureFlywheelReady",
                () -> {
                    if (launcherVelocity < closeLauncherPower) {
                        setCloseLaunchPower();
                    }
                },
                this::flywheelReady);
    }

    private Step launchCloseTopGate() {
        return new Step(
                "launchCloseTopGate",
                () -> {
                    topGatePosition = topGateClosedPosition;
                },
                Step.secondsElapsed(0.05),
                clock);
    }

    private Step launchOpenBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateOpenPosition;
                },
                Step.secondsElapsed(0.05),
                clock);
    }

    private Step launchCloseBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateClosedPosition;
                },
                Step.secondsElapsed(0.08),
                clock);
    }

    private Step launchOpenTopGate() {
        return new Step(
                "launchOpenTopGate",
                () -> {
                    topGatePosition = topGateOpenPosition;
                },
                Step.secondsElapsed(0.05),
                clock);
    }

    public void slowLaunchyLaunch() {
        if (planRunner.done()) {
            planRunner.run(slowLaunchPlan());
        }
    }

    private Plan slowLaunchPlan() {
        return new Plan(launchPlan(), Step.waitFor("slow launch", 0.8, clock));
    }

    public void increasePower() {
        launcherVelocity = launcherVelocity + 25;
        if (launcherVelocity > 10000) {
            launcherVelocity = 10000;
        }
    }

    public void decreasePower() {
        launcherVelocity = launcherVelocity - 25;
        if (launcherVelocity < -10000) {
            launcherVelocity = -10000;
        }
    }

    public void setCloseLaunchPower() {
        launcherVelocity = closeLauncherPower;
    }

    /** Holds the bottom gate open a little longer on the next launch. */
    public void increaseBottomGateWaitTime() {
        bottomGateWaitSeconds = bottomGateWaitSeconds + BOTTOM_GATE_WAIT_STEP_SECONDS;
    }

    /** Holds it open a little less; never below nothing, which would close it the loop it opened. */
    public void decreaseBottomGateWaitTime() {
        bottomGateWaitSeconds = Math.max(0, bottomGateWaitSeconds - BOTTOM_GATE_WAIT_STEP_SECONDS);
    }

    /** How long the next launch holds the bottom gate open, in seconds. */
    public double bottomGateWaitSeconds() {
        return bottomGateWaitSeconds;
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("launcherStep", planRunner.currentStep());

        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("launcherVelocityTarget", launcherVelocity);
        telemetry.addData("launcherVelocityActual", launcher.getVelocity());
        telemetry.addData("topGatePosition", topGatePosition);
        telemetry.addData("bottomGatePosition", bottomGatePosition);
        telemetry.addData("bottomGateWaitSeconds", bottomGateWaitSeconds);
    }

    public boolean launchDone() {
        return planRunner.done();
    }

    private boolean closeEnough(double a, double b, double c) {
        return Math.abs(a - b) < c;
    }

    public boolean flywheelReady() {
        return launcherOn() && closeEnough(launcher.getVelocity(), launcherVelocity, 15);
    }

    private boolean launcherOn() {
        return launcherVelocity > 0;
    }
}
