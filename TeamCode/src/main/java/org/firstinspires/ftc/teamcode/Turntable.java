package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.SubSystem;

public class Turntable extends SubSystem {
    public static final int TICKS_PER_REVOLUTION = 1700;
    private final DcMotorEx turnTable;
    private boolean telemetryOn = false;
    private int turnTableTargetPosition;

    public Turntable(DcMotorEx turnTable) {
        this.turnTable = turnTable;
    }

    @Override
    protected void onLoop() {
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

    public void turnTableToLeft() {
        turnTableTargetPosition += 10;
    }

    public void turnTableToRight() {
        turnTableTargetPosition -= 10;
    }

    public double getTurnTableOffsetRadians() {
        double revolutions = (double) turnTable.getCurrentPosition() / TICKS_PER_REVOLUTION;
        return revolutions * (Math.PI * 2);
    }

    public void setTurnTablePosition(double relativeHeadingRadians) {
        double radiansPerRevolution = Math.PI * 2;
        double ticksPerRadian = TICKS_PER_REVOLUTION / radiansPerRevolution;
        int rawTurnTableTargetPosition = (int) (relativeHeadingRadians * ticksPerRadian);
        int middleTurnTablePosition = rawTurnTableTargetPosition % TICKS_PER_REVOLUTION;

        if (middleTurnTablePosition > TICKS_PER_REVOLUTION / 2) {
            turnTableTargetPosition = middleTurnTablePosition - TICKS_PER_REVOLUTION;
        } else if (middleTurnTablePosition < -TICKS_PER_REVOLUTION / 2) {
            turnTableTargetPosition = middleTurnTablePosition + TICKS_PER_REVOLUTION;
        } else {
            turnTableTargetPosition = middleTurnTablePosition;
        }
    }
}
