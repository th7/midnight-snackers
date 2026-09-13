package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Launcher extends SubSystem {
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
    private boolean telemetryOn = false;
    private double bottomGateWaitTime = 0.45;
    private final PlanRunner planRunner = add(new PlanRunner());

    public Launcher(DcMotorEx launcher, Servo topGate, Servo bottomGate) {
        this.launcher = launcher;
        this.topGate = topGate;
        this.bottomGate = bottomGate;
    }

    @Override
    public void init() {
        launcher.setPositionPIDFCoefficients(5);

        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);
    }

    @Override
    protected void onLoop() {
        launcher.setVelocity(launcherVelocity);
        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);

        if (telemetryOn) {
            setTelemetry();
        }
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
                Step.waitFor("ball to fall into launcher", 0.15),
                launchCloseBottomGate(),
                launchOpenTopGate(),
                Step.waitFor("ball to fall into bottom position", 0.15)
        );
    }

    private Step ensureFlywheelReady() {
        return new Step(
                "ensureFlywheelReady",
                () -> {
                    if (launcherVelocity < closeLauncherPower) {
                        setCloseLaunchPower();
                    }
                },
                this::flywheelReady
        );
    }

    private Step launchCloseTopGate() {
        return new Step(
                "launchCloseTopGate",
                () -> {
                    topGatePosition = topGateClosedPosition;
                },
                Step.secondsElapsed(0.05)
        );
    }

    private Step launchOpenBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateOpenPosition;
                },
                Step.secondsElapsed(0.05)
        );
    }

    private Step launchCloseBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateClosedPosition;
                },
                Step.secondsElapsed(0.08)
        );
    }

    private Step launchOpenTopGate() {
        return new Step(
                "launchOpenTopGate",
                () -> {
                    topGatePosition = topGateOpenPosition;
                },
                Step.secondsElapsed(0.05)
        );
    }

    public void slowLaunchyLaunch() {
        if (planRunner.done()) {
            planRunner.run(slowLaunchPlan());
        }
    }

    private Plan slowLaunchPlan() {
        return new Plan(
                launchPlan(),
                Step.waitFor("slow launch", 0.8)
        );
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

    public void increaseBottomGateWaitTime() {
        bottomGateWaitTime = bottomGateWaitTime + 0.0001;
    }

    public void decreaseBottomGateWaitTime() {
        bottomGateWaitTime = bottomGateWaitTime - 0.0001;
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }

    private void setTelemetry() {
        telemetry.addData("Launcher", "telemetry on");
        telemetry.addData("launcherStep", planRunner.currentStep());

        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("launcherVelocityTarget", launcherVelocity);
        telemetry.addData("launcherVelocityActual", launcher.getVelocity());
        telemetry.addData("topGatePosition", topGatePosition);
        telemetry.addData("bottomGatePosition", bottomGatePosition);
        telemetry.addData("bottomGateWaitTime", bottomGateWaitTime);
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
