package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private final double rangedLauncherPower = 1350d;
    private final DcMotorEx launcher;
    private final Servo topGate;
    private final Servo bottomGate; // bottomGate is closer to launcher
    private double topGatePosition = topGateOpenPosition;
    private double bottomGatePosition = bottomGateClosedPosition;
    private double launcherVelocity = 0d;
    private boolean telemetryOn = false;
    private double bottomGateWaitTime = 0.45;
    private double topGateWaitTime = bottomGateWaitTime;
    private PIDFCoefficients pidVelocityOrig;
    private PIDFCoefficients pidOrig;
    private double PIDFAdjustable = 0;
    private final PlanRunner planRunner = new PlanRunner();

    public Launcher(DcMotorEx launcher, Servo topGate, Servo bottomGate, ElapsedTime runtime, Telemetry telemetry) {
        super(runtime, telemetry);
        this.launcher = launcher;
        this.topGate = topGate;
        this.bottomGate = bottomGate;
    }

    public void init() {
        pidOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        launcher.setPositionPIDFCoefficients(5);

        pidVelocityOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);

        telemetry.addData("Launcher.init()", true);
    }

    @Override
    public void loop() {
        planRunner.loop();

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

    public void setFarLaunchPower() {
        launcherVelocity = rangedLauncherPower;
    }

    public void noPower() {
        launcherVelocity = 0;
    }

    public void increaseBottomGatePosition() {
        bottomGatePosition = bottomGatePosition + 0.05;
    }

    public void decreaseBottomGatePosition() {
        bottomGatePosition = bottomGatePosition - 0.05;
    }

    public void increaseTopGatePosition() {
        topGatePosition = topGatePosition + 0.05;
    }

    public void decreaseTopGatePosition() {
        topGatePosition = topGatePosition - 0.05;
    }

    public void increaseTopGateWaitTime() {
        topGateWaitTime = topGateWaitTime + 0.0001;
    }

    public void decreaseTopGateWaitTime() {
        topGateWaitTime = topGateWaitTime - 0.0001;
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

//        telemetry.addData("adjustable", PIDFAdjustable);
        telemetry.addData("launcherPower", launcher.getPower());
//        telemetry.addData("launcherTargetPosition", launcher.getTargetPosition());
        telemetry.addData("launcherVelocityTarget", launcherVelocity);
        telemetry.addData("launcherVelocityActual", launcher.getVelocity());
        telemetry.addData("topGatePosition", topGatePosition);
        telemetry.addData("topGateWaitTime", topGateWaitTime);
        telemetry.addData("bottomGatePosition", bottomGatePosition);
        telemetry.addData("bottomGateWaitTime", bottomGateWaitTime);
        telemetry.addData("PIDF vel (orig)", "%.04f, %.04f, %.04f, %.04f",
                pidVelocityOrig.p, pidVelocityOrig.i, pidVelocityOrig.d, pidVelocityOrig.f);
        PIDFCoefficients pidVelocityModified = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF vel (modified)", "%.04f, %.04f, %.04f, %.04f",
                pidVelocityModified.p, pidVelocityModified.i, pidVelocityModified.d, pidVelocityModified.f);
        telemetry.addData("PIDF (orig)", "%.04f, %.04f, %.04f, %.04f",
                pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
        PIDFCoefficients pidModified = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("PIDF (modified)", "%.04f, %.04f, %.04f, %.04f",
                pidModified.p, pidModified.i, pidModified.d, pidModified.f);
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

//    public void launchMotifFirst(Plans.Motif motif) {
//        if (motif == Plans.Motif.GPP) {
//            gate1Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PGP) {
//            gate2Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PPG) {
//            gate3Position = gate2OpenPosition;
//        } else {
//            gate1Position = gate2OpenPosition;
//        }
//        launchStartedAt = runtime.time();
//    }
//
//    public void launchMotifSecond(Plans.Motif motif) {
//        if (motif == Plans.Motif.GPP) {
//            gate2Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PGP) {
//            gate1Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PPG) {
//            gate3Position = gate2OpenPosition;
//        } else {
//            gate2Position = gate2OpenPosition;
//        }
//        launchStartedAt = runtime.time();
//    }
//
//    public void launchMotifThird(Plans.Motif motif) {
//        if (motif == Plans.Motif.GPP) {
//            gate3Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PGP) {
//            gate3Position = gate2OpenPosition;
//        } else if (motif == Plans.Motif.PPG) {
//            gate1Position = gate2OpenPosition;
//        } else {
//            gate3Position = gate2OpenPosition;
//        }
//        launchStartedAt = runtime.time();
//    }

    public void increaseAdjustable() {
        PIDFAdjustable = PIDFAdjustable + 0.1;
    }

    public void decreaseAdjustable() {
        PIDFAdjustable = PIDFAdjustable - 0.1;
    }
}
