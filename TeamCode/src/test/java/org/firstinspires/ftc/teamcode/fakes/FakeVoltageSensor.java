package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class FakeVoltageSensor implements VoltageSensor {
    public double voltage;

    public FakeVoltageSensor(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public double getVoltage() {
        return voltage;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "fake voltage sensor";
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
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }
}
