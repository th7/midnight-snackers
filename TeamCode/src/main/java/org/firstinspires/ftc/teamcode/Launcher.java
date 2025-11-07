package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SubSystem;

public class Launcher extends SubSystem {
    private final double gateOpenPosition = 1;
    private final double gateClosedPosition = 0;
    private final double closeLauncherPower = 1050d;
    private final double rangedLauncherPower = 1350d;
    private DcMotorEx launcher;
    private Servo gate;
    private double gatePosition;
    private double launcherVelocity = 0d;
    private double launchStartedAt = -1;
    private boolean telemetryOn = false;
    private double gate1Position;
    private double gate2Position;
    private double gate3Position;
    private double gateWaitTime = 2;
    private PIDFCoefficients pidVelocityOrig;
    private PIDFCoefficients pidOrig;

    private double adjustable = 0;

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

        gate = hardwareMap.get(Servo.class, "gate");
        telemetry.addData("Launcher.init()", true);
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

    public void launch() {
        if (flywheelReady() && gatePosition == gateClosedPosition) {
            launchNow();
        }
    }

    public void increaseGatePosition() {
        gatePosition = gatePosition + 0.05;
    }

    public void decreaseGatePosition() {
        gatePosition = gatePosition - 0.05;
    }

    public void increaseGateWaitTime() {
        gateWaitTime = gateWaitTime + 0.01;
    }

    public void decreaseGateWaitTime() {
        gateWaitTime = gateWaitTime - 0.01;
    }

    @Override
    public void loop() {
        launcher.setVelocity(launcherVelocity);

        if (gatePosition == gateOpenPosition && gateWaitTimePassed()) {
            gatePosition = gateClosedPosition;
        }

        gate.setPosition(gatePosition);

        if (telemetryOn) { setTelemetry(); }
    }

    public void toggleTelemetry() { telemetryOn = !telemetryOn; }

    private void setTelemetry() {
        telemetry.addData("adjustable", adjustable);
        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("launcherTargetPosition", launcher.getTargetPosition());
        telemetry.addData("launcherVelocityTarget", launcherVelocity);
        telemetry.addData("launcherVelocityActual", launcher.getVelocity());
        telemetry.addData("gatePosition", gatePosition);
        telemetry.addData("gateWaitTime", gateWaitTime);
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

    private boolean gateWaitTimePassed() {
        return runtime.time() >= launchStartedAt + gateWaitTime;
    }

    @Override
    public boolean done() {
        return gateWaitTimePassed();
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

    public void launchMotifFirst(Plans.Motif motif) {
        if (motif == Plans.Motif.GPP) {
            gate1Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PGP) {
            gate2Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PPG) {
            gate3Position = gateOpenPosition;
        } else {
            gate1Position = gateOpenPosition;
        }
        launchStartedAt = runtime.time();
    }

    public void launchMotifSecond(Plans.Motif motif) {
        if (motif == Plans.Motif.GPP) {
            gate2Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PGP) {
            gate1Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PPG) {
            gate3Position = gateOpenPosition;
        } else {
            gate2Position = gateOpenPosition;
        }
        launchStartedAt = runtime.time();
    }

    public void launchMotifThird(Plans.Motif motif) {
        if (motif == Plans.Motif.GPP) {
            gate3Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PGP) {
            gate3Position = gateOpenPosition;
        } else if (motif == Plans.Motif.PPG) {
            gate1Position = gateOpenPosition;
        } else {
            gate3Position = gateOpenPosition;
        }
        launchStartedAt = runtime.time();
    }

    public void increaseAdjustable() {
        adjustable = adjustable + 0.1;
//        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);
//        launcher.setPositionPIDFCoefficients(5);
    }
    public void decreaseAdjustable() {
        adjustable = adjustable - 0.1;
//        launcher.setVelocityPIDFCoefficients(250, 0, 0, 12.9);
//        launcher.setPositionPIDFCoefficients(5);
    }

    public boolean stopped() {
        return launcherVelocity < 1;
    }

    public void launchNow() {
        gatePosition = gateOpenPosition;
        launchStartedAt = runtime.time();
    }
}
