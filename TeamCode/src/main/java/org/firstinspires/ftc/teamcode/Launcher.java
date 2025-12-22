package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Launcher extends SubSystem {
    private final double topGateOpenPosition = 1;
    private final double topGateClosedPosition = 0.6;
    private final double bottomGateOpenPosition = 0.5;
    private final double bottomGateClosedPosition = 0.4;
    private final double closeLauncherPower = 1050d;
    private final double rangedLauncherPower = 1350d;
    private DcMotorEx launcher;
    private Servo topGate;
    private Servo bottomGate; // bottomGate is closer to launcher
    private double topGatePosition = topGateOpenPosition;
    private double bottomGatePosition = bottomGateClosedPosition;
    private double launcherVelocity = 0d;
    private double bottomGateStartedAt = -1;
    private double topGateStartedAt = -1;
    private double waitForStartedAt = -1;
    private boolean telemetryOn = false;
    private boolean loading = false;
    private double bottomGateWaitTime = 0.45;
    private double topGateWaitTime = bottomGateWaitTime;
    private PIDFCoefficients pidVelocityOrig;
    private PIDFCoefficients pidOrig;
    private double PIDFAdjustable = 0;
    private Plan currentPlan = null;

    public Launcher(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        pidOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        launcher.setPositionPIDFCoefficients(5);

        pidVelocityOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topGate = hardwareMap.get(Servo.class, "topGate");
        bottomGate = hardwareMap.get(Servo.class, "bottomGate");
        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);

        telemetry.addData("Launcher.init()", true);
    }

    @Override
    public void loop() {
        if (currentPlan != null && currentPlan.done()) {
            currentPlan = null;
        }

        launcher.setVelocity(launcherVelocity);
        topGate.setPosition(topGatePosition);
        bottomGate.setPosition(bottomGatePosition);

        if (telemetryOn) {
            setTelemetry();
        }
    }

    public void launchyLaunch() {
        if (currentPlan == null) {
            currentPlan = launchPlan();
        }
    }

    private Plan launchPlan() {
        return new Plan(
                ensureFlywheelReady(),
                launchCloseTopGate(),
                launchOpenBottomGate(),
                launchCloseBottomGate(),
                launchOpenTopGate()
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
                    topGateStartedAt = runtime.time();
                },
                () -> this.runtime.time() >= topGateStartedAt + 0.05
        );
    }

    private Step launchOpenBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateOpenPosition;
                    bottomGateStartedAt = runtime.time();
                },
                () -> this.runtime.time() >= bottomGateStartedAt + 0.2
        );
    }

    private Step launchCloseBottomGate() {
        return new Step(
                "launchOpenBottomGate",
                () -> {
                    bottomGatePosition = bottomGateClosedPosition;
                    bottomGateStartedAt = runtime.time();
                },
                () -> this.runtime.time() >= bottomGateStartedAt + 0.08
        );
    }

    private Step launchOpenTopGate() {
        return new Step(
                "launchOpenTopGate",
                () -> {
                    topGatePosition = topGateOpenPosition;
                    topGateStartedAt = runtime.time();
                },
                () -> true
        );
    }

    public void slowLaunchyLaunch() {
        if (currentPlan == null) {
            currentPlan = slowLaunchPlan();
        }
    }

    private Plan slowLaunchPlan() {
        return new Plan(
                launchPlan(),
                waitFor(0.8)
        );
    }

    private Step waitFor(double seconds) {
        return new Step(
                "waitFor",
                () -> {
                    waitForStartedAt = runtime.time();
                },
                () -> runtime.time() >= waitForStartedAt + seconds
        );
    }

    public void loadyLoad() {
        if (currentPlan == null) {
            currentPlan = loadPlan();
        }
    }

    public void finishLoading() {
        loading = false;
    }

    private Plan loadPlan() {
        return new Plan(
                loadOpenGate1(),
                loadCloseGate1()
        );
    }

    private Step loadOpenGate1() {
        return new Step(
                "loadOpenGate1",
                () -> {
                    loading = true;
                    topGatePosition = topGateOpenPosition;
                },
                () -> {
                    return !loading;
                }
        );
    }

    private Step loadCloseGate1() {
        return new Step(
                "loadCloseGate1",
                () -> {
                    topGatePosition = topGateClosedPosition;
                    topGateStartedAt = runtime.time();
                },
                this::topGateWaitTimePassed
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
        if (currentPlan != null) {
            telemetry.addData("launcherStep", currentPlan.currentStep());
        } else {
            telemetry.addData("launcherStep", "no currentPlan");
        }

        telemetry.addData("adjustable", PIDFAdjustable);
        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("launcherTargetPosition", launcher.getTargetPosition());
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

    private boolean bottomGateWaitTimePassed() {
        return runtime.time() >= bottomGateStartedAt + bottomGateWaitTime;
    }

    private boolean topGateWaitTimePassed() {
        return runtime.time() >= topGateStartedAt + topGateWaitTime;
    }

    private boolean topGateLaunchFinished() {
        return runtime.time() >= topGateStartedAt + topGateWaitTime * 2 + 0.05;
    }

    private boolean bottomGateLaunchFinished() {
        return runtime.time() >= bottomGateStartedAt + bottomGateWaitTime * 2 + 0.05;
    }

    public boolean gateLaunchDone() {
        return topGateLaunchFinished() && bottomGateLaunchFinished();
    }

    public boolean launchDone() {
        return gateLaunchDone() && currentPlan == null;
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
