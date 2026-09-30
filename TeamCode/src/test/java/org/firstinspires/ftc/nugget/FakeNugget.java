package org.firstinspires.ftc.nugget;

import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;

public final class FakeNugget {
    public static final double BATTERY_VOLTS = 12;

    public final FakeDcMotorEx left = new FakeDcMotorEx();
    public final FakeDcMotorEx right = new FakeDcMotorEx();
    public final FakeVoltageSensor battery = new FakeVoltageSensor(BATTERY_VOLTS);
    public final FakeDashboard dashboard = new FakeDashboard();
    public long nanos = 0;

    public NuggetHardware hardware() {
        return NuggetHardware.builder()
                .left(left)
                .right(right)
                .voltageSensor(battery)
                .dashboard(dashboard)
                .nanoClock(() -> nanos)
                .build();
    }
}
