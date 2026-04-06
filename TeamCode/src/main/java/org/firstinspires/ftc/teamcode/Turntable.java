package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SubSystem;

public class Turntable extends SubSystem {
    private final int ticksPerRevolution = 1700;
    private DcMotorEx turnTable;
    private boolean telemetryOn = false;
    private int turnTableTargetPosition;

    public Turntable(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        turnTable = hardwareMap.get(DcMotorEx.class, "turnTable");
        telemetry.addData("Turntable.init()", true);
    }

    @Override
    public void loop() {
        double turnTableError = turnTableTargetPosition - turnTable.getCurrentPosition();
        double turnTablePower = turnTableError / 100;

        if (turnTableError < 5 && turnTableError > -5) {
            turnTable.setPower(0);
        } else {
            double finalTurnTablePower = clampMinPower(turnTablePower, 0.1);
            turnTable.setPower(finalTurnTablePower);
        }

        if (telemetryOn) {
            setTelemetry();
        }
    }

    private float clampMinPower(double power, double min) {
        if (power > 0 && power < min) {
            return (float) min;
        } else if (power < 0 && power > -min) {
            return (float) -min;
        } else {
            return (float) power;
        }
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }

    private void setTelemetry() {
        telemetry.addData("Turntable", "telemetry on");

        telemetry.addData("turnTableRotationTicks", turnTable.getCurrentPosition());
        telemetry.addData("turnTableOffsetRadians", getTurnTableOffsetRadians());
        telemetry.addData("turnTableTargetPosition", turnTableTargetPosition);
        telemetry.addData("turnTablePower", turnTable.getPower());
    }

    private boolean closeEnough(double a, double b, double c) {
        return Math.abs(a - b) < c;
    }

    public void turnTableToLeft() {
        turnTableTargetPosition += 10;
    }

    public void turnTableToRight() {
        turnTableTargetPosition -= 10;
    }

    public double getTurnTableOffsetRadians() {
        double revolutions = (double) turnTable.getCurrentPosition() / ticksPerRevolution;
        return revolutions * (Math.PI * 2);
    }

    public void setTurnTablePosition(double relativeHeadingRadians) {
        double radiansPerRevolution = Math.PI * 2;
        double ticksPerRadian = ticksPerRevolution / radiansPerRevolution;
        int rawTurnTableTargetPosition = (int) (relativeHeadingRadians * ticksPerRadian);
        int middleTurnTablePosition = rawTurnTableTargetPosition % ticksPerRevolution;

        if (middleTurnTablePosition > ticksPerRevolution / 2) {
            turnTableTargetPosition = middleTurnTablePosition - ticksPerRevolution;
        } else if (middleTurnTablePosition < -ticksPerRevolution / 2) {
            turnTableTargetPosition = middleTurnTablePosition + ticksPerRevolution;
        } else {
            turnTableTargetPosition = middleTurnTablePosition;
        }
    }
}
