package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private double launcherPower = 0d;
    private double launchStartedAt = -1;
    private boolean telemetryOn = false;
    private double gate1Position;
    private double gate2Position;
    private double gate3Position;
    private double gateWaitTime = 0.38;

    public Launcher(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        launcher.setVelocityPIDFCoefficients(0.01, 1, 50, 0.01);
        gate = hardwareMap.get(Servo.class, "gate");
        telemetry.addData("Launcher.init()", true);
    }

    public void increasePower() {
        launcherPower = launcherPower + 25;
        if (launcherPower > 10000) {
            launcherPower = 10000;
        }
    }

    public void decreasePower() {
        launcherPower = launcherPower - 25;
        if (launcherPower < -10000) {
            launcherPower = -10000;
        }
    }

    public void setCloseLaunchPower() {
        launcherPower = closeLauncherPower;
    }

    public void setFarLaunchPower() {
        launcherPower = rangedLauncherPower;
    }

    public void noPower() {
        launcherPower = 0;
    }

    public void launch() {
        gatePosition = gateOpenPosition;
        launchStartedAt = runtime.time();
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
        launcher.setVelocityPIDFCoefficients(0.0001, 1, 1, 1);
        launcher.setVelocity(launcherPower);

        if (gatePosition == gateOpenPosition && secondAfterGateOpen()) {
            gatePosition = gateClosedPosition;
        }

        gate.setPosition(gatePosition);

        if (telemetryOn) { setTelemetry(); }
    }

    public void toggleTelemetry() { telemetryOn = !telemetryOn; }

    private void setTelemetry() {
        telemetry.addData("launcherPower", launcherPower);
        telemetry.addData("launcherVelocity", launcher.getVelocity());
        telemetry.addData("gatePosition", gatePosition);
        telemetry.addData("gateWaitTime", gateWaitTime);
//        telemetry.addData("velocityPIDFCoefficients", launcher.getPIDFCoefficients());
    }

    private boolean secondAfterGateOpen() {
        return runtime.time() >= launchStartedAt + gateWaitTime;
    }

    @Override
    public boolean done() {
        return secondAfterGateOpen();
    }

    private boolean closeEnough(double a, double b, double c) {
        return Math.abs(a - b) < c;
    }

    public boolean flywheelReady() {
        return closeEnough(launcher.getVelocity(), launcherPower, 15);
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
}
