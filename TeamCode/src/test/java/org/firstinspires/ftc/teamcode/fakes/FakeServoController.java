package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import java.util.HashMap;
import java.util.Map;

public class FakeServoController implements ServoControllerEx {
    private final Map<Integer, Double> positions = new HashMap<>();
    private final Map<Integer, PwmControl.PwmRange> ranges = new HashMap<>();
    private final Map<Integer, Boolean> enabled = new HashMap<>();

    public double position(int port) {
        return positions.getOrDefault(port, 0.0);
    }

    public boolean driven(int port) {
        return positions.containsKey(port) && isServoPwmEnabled(port);
    }

    @Override
    public void pwmEnable() {}

    @Override
    public void pwmDisable() {}

    @Override
    public PwmStatus getPwmStatus() {
        return PwmStatus.ENABLED;
    }

    @Override
    public void setServoPosition(int servo, double position) {
        positions.put(servo, position);
    }

    @Override
    public double getServoPosition(int servo) {
        return position(servo);
    }

    @Override
    public void forgetLastKnownPosition(int servo) {}

    @Override
    public void setServoPwmRange(int servo, PwmControl.PwmRange range) {
        ranges.put(servo, range);
    }

    @Override
    public PwmControl.PwmRange getServoPwmRange(int servo) {
        return ranges.getOrDefault(servo, PwmControl.PwmRange.defaultRange);
    }

    @Override
    public void setServoPwmEnable(int servo) {
        enabled.put(servo, true);
    }

    @Override
    public void setServoPwmDisable(int servo) {
        enabled.put(servo, false);
    }

    @Override
    public boolean isServoPwmEnabled(int servo) {
        return enabled.getOrDefault(servo, true);
    }

    @Override
    public void setServoType(int servo, ServoConfigurationType servoType) {}

    @Override
    public void setPulseWidth(int servo, double usPulse) {}

    @Override
    public double getPulseWidth(int servo) {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "fake servo controller";
    }

    @Override
    public String getConnectionInfo() {
        return "fake";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {}
}
