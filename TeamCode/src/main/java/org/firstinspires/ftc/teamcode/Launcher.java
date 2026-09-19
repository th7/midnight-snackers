package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Launcher implements Loopable {
    public static final String CHANNEL = "Launcher";

    private final Prints telemetry;

    public static final double BOTTOM_GATE_WAIT_SECONDS = 0.15;

    public static final double BOTTOM_GATE_WAIT_STEP_SECONDS = 0.01;

    private final LongSupplier nanoClock;

    private final double topGateOpenPosition = 1;
    private final double topGateClosedPosition = 0.6;
    private final double bottomGateOpenPosition = 0.5;
    private final double bottomGateClosedPosition = 0.4;
    private final double closeLauncherPower = 1050d;
    private final DcMotorEx launcher;
    private final Servo topGate;
    private final Servo bottomGate;
    private double topGatePosition = topGateOpenPosition;
    private double bottomGatePosition = bottomGateClosedPosition;
    private double launcherVelocity = 0d;
    private double bottomGateWaitSeconds = BOTTOM_GATE_WAIT_SECONDS;
    private final PlanRunner planRunner = new PlanRunner();

    public Launcher(DcMotorEx launcher, Servo topGate, Servo bottomGate, LongSupplier nanoClock, Prints telemetry) {
        this.nanoClock = nanoClock;
        this.launcher = launcher;
        this.topGate = topGate;
        this.bottomGate = bottomGate;
        this.telemetry = telemetry;

        launcher.setPositionPIDFCoefficients(5);
        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                Step.waitFor("ball to fall into launcher", bottomGateWaitSeconds, nanoClock),
                launchCloseBottomGate(),
                launchOpenTopGate(),
                Step.waitFor("ball to fall into bottom position", 0.15, nanoClock));
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
                nanoClock);
    }

    private Step launchOpenBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateOpenPosition;
                },
                Step.secondsElapsed(0.05),
                nanoClock);
    }

    private Step launchCloseBottomGate() {
        return new Step(
                "launchCloseBottomGate",
                () -> {
                    bottomGatePosition = bottomGateClosedPosition;
                },
                Step.secondsElapsed(0.08),
                nanoClock);
    }

    private Step launchOpenTopGate() {
        return new Step(
                "launchOpenTopGate",
                () -> {
                    topGatePosition = topGateOpenPosition;
                },
                Step.secondsElapsed(0.05),
                nanoClock);
    }

    public void slowLaunchyLaunch() {
        if (planRunner.done()) {
            planRunner.run(slowLaunchPlan());
        }
    }

    private Plan slowLaunchPlan() {
        return new Plan(launchPlan(), Step.waitFor("slow launch", 0.8, nanoClock));
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

    public void increaseBottomGateWaitSeconds() {
        bottomGateWaitSeconds = bottomGateWaitSeconds + BOTTOM_GATE_WAIT_STEP_SECONDS;
    }

    public void decreaseBottomGateWaitSeconds() {
        bottomGateWaitSeconds = Math.max(0, bottomGateWaitSeconds - BOTTOM_GATE_WAIT_STEP_SECONDS);
    }

    public double bottomGateWaitSeconds() {
        return bottomGateWaitSeconds;
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

    @Override
    public void loop() {
        planRunner.loop();
        launcher.setVelocity(launcherVelocity);
        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);

        telemetry.addData("launcherStep", planRunner.currentStep());

        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("launcherVelocityTarget", launcherVelocity);
        telemetry.addData("launcherVelocityActual", launcher.getVelocity());
        telemetry.addData("topGatePosition", topGatePosition);
        telemetry.addData("bottomGatePosition", bottomGatePosition);
        telemetry.addData("bottomGateWaitSeconds", bottomGateWaitSeconds);
    }
}
