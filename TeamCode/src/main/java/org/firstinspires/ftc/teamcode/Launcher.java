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
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Launcher extends SubSystem {
    private final double gateOpenPosition = 1;
    private final double gateClosedPosition = 0;
    private final double closeLauncherPower = 1050d;
    private final double rangedLauncherPower = 1350d;
    private DcMotorEx launcher;
    private Servo gate1;
    private Servo gate2; //gate2 is closer to launcher
    private double gate1Position;
    private double gate2Position;
    private double launcherVelocity = 0d;
    private double gate2StartedAt = -1;
    private double gate1StartedAt = -1;
    private boolean telemetryOn = false;
    private boolean loading = false;
    private double gate2WaitTime = 0.45;
    private double gate1WaitTime = gate2WaitTime;
    private PIDFCoefficients pidVelocityOrig;
    private PIDFCoefficients pidOrig;
    private double PIDFAdjustable = 0;
    private boolean launching;
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

        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate2 = hardwareMap.get(Servo.class, "gate2");
        gate1.setPosition(gateClosedPosition);
        gate2.setPosition(gateClosedPosition);

        telemetry.addData("Launcher.init()", true);
    }

    @Override
    public void loop() {
        if (currentPlan != null) {
            if (currentPlan.done()) { currentPlan = null; }
        }

        launcher.setVelocity(launcherVelocity);
        gate1.setPosition(gate1Position);
        gate2.setPosition(gate2Position);

        if (telemetryOn) { setTelemetry(); }
    }

    public void launchyLaunch() {
        if (currentPlan == null) {
            currentPlan = launchPlan();
        }
    }

    private Plan launchPlan() {
        return new Plan(
                ensureFlywheelReady(),
                launchOpenGate2(),
                launchCloseGate2OpenGate1(),
                launchCloseGate1()
        );
    }

    private Step ensureFlywheelReady() {
        return new Step(
                "ensureFlywheelReady",
                this::setCloseLaunchPower,
                this::flywheelReady
        );
    }

    private Step launchOpenGate2() {
        return new Step(
                "launchOpenGate2",
                () -> {
                    gate2Position = gateOpenPosition;
                    gate2StartedAt = runtime.time();
                },
                this::gate2WaitTimePassed
        );
    }

    private Step launchCloseGate2OpenGate1() {
        return new Step(
                "launchCloseGate2OpenGate1",
                () -> {
                    gate2Position = gateClosedPosition;
                    gate1Position = gateOpenPosition;
                    gate1StartedAt = runtime.time();
                },
                this::gate1WaitTimePassed
        );
    }

    private Step launchCloseGate1() {
        return new Step(
                "launchCloseGate1",
                () -> {
                    gate1Position = gateClosedPosition;
                },
                this::gateLaunchDone
        );
    }

    public void loadyLoad() {
        if (currentPlan == null) { currentPlan = loadPlan(); }
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
                    gate1Position = gateOpenPosition;
                },
                () -> { return !loading; }
        );
    }

    private Step loadCloseGate1() {
        return new Step(
                "loadCloseGate1",
                () -> {
                    gate1Position = gateClosedPosition;
                    gate1StartedAt = runtime.time();
                },
                this::gate1WaitTimePassed
        );
    }

    public void launch() {
        if (flywheelStopped()) {
            setCloseLaunchPower();
        } else if (flywheelReady() && !loading && gate2Position == gateClosedPosition) {
            launchNow();
        }
    }

    public void launchNow() {
        gate2Position = gateOpenPosition;
        gate2StartedAt = runtime.time();
        launching = true;
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

    public void increaseGatePosition() {
        gate2Position = gate2Position + 0.05;
    }

    public void decreaseGatePosition() {
        gate2Position = gate2Position - 0.05;
    }

    public void increaseGateWaitTime() {
        gate2WaitTime = gate2WaitTime + 0.0001;
    }

    public void decreaseGateWaitTime() {
        gate2WaitTime = gate2WaitTime - 0.0001;
    }

//    public void startLoading() {
//        if (!launching) {
//            loading = true;
//        }
//    }

    public void toggleTelemetry() { telemetryOn = !telemetryOn; }

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
        telemetry.addData("gate1Position", gate1Position);
        telemetry.addData("gate1WaitTime", gate1WaitTime);
        telemetry.addData("gate2Position", gate2Position);
        telemetry.addData("gate2WaitTime", gate2WaitTime);
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

//        telemetry.addData("velocityPIDFCoefficients", launcher.getPIDFCoefficients());
    }

    private boolean gate2WaitTimePassed() {
        return runtime.time() >= gate2StartedAt + gate2WaitTime;
    }

    private boolean gate1WaitTimePassed() {
        return runtime.time() >= gate1StartedAt + gate1WaitTime;
    }

    private boolean gate1LaunchFinished() {
        return runtime.time() >= gate1StartedAt + gate1WaitTime * 2 + 0.05;
    }

    private boolean gate2LaunchFinished() {
        return runtime.time() >= gate2StartedAt + gate2WaitTime * 2 + 0.05;
    }

//    @Override
    public boolean gateLaunchDone() {
        return gate1LaunchFinished() && gate2LaunchFinished();
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
//        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);
//        launcher.setPositionPIDFCoefficients(5);
    }
    public void decreaseAdjustable() {
        PIDFAdjustable = PIDFAdjustable - 0.1;
//        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);
//        launcher.setPositionPIDFCoefficients(5);
    }

    public boolean flywheelStopped() {
        return launcherVelocity < 1;
    }
}
