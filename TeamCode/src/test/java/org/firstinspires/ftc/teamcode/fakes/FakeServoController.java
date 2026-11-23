package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.PulseRange;
import org.firstinspires.ftc.teamcode.simcore.ServoPort;
import org.firstinspires.ftc.teamcode.simcore.ServoSignal;

public class FakeServoController implements ServoControllerEx {
    private static final Range SDK_DEFAULT = Range.of(PwmControl.PwmRange.defaultRange);

    private final Map<Integer, ServoPort> ports = new HashMap<>();
    private final Map<Integer, Range> ranges = new HashMap<>();

    private record Range(PwmControl.PwmRange sdk, PulseRange pulses) {
        static Range of(PwmControl.PwmRange sdk) {
            return new Range(sdk, accepted(PulseRange.of(sdk.usPulseLower, sdk.usPulseUpper)));
        }
    }

    private static <T> T accepted(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new IllegalArgumentException(rule);
        });
    }

    public ServoSignal signal(int servo) {
        return port(servo).signal();
    }

    private ServoPort port(int servo) {
        return ports.getOrDefault(servo, ServoPort.unwritten());
    }

    private Range range(int servo) {
        return ranges.getOrDefault(servo, SDK_DEFAULT);
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
        ports.put(servo, accepted(port(servo).positioned(position, range(servo).pulses())));
    }

    @Override
    public double getServoPosition(int servo) {
        ServoPort.PositionRead read = port(servo).read(range(servo).pulses());
        ports.put(servo, read.port());
        return read.position();
    }

    @Override
    public void forgetLastKnownPosition(int servo) {
        ports.put(servo, port(servo).forgotten());
    }

    @Override
    public void setServoPwmRange(int servo, PwmControl.PwmRange range) {
        ranges.put(servo, Range.of(range));
    }

    @Override
    public PwmControl.PwmRange getServoPwmRange(int servo) {
        return range(servo).sdk();
    }

    @Override
    public void setServoPwmEnable(int servo) {
        ports.put(servo, port(servo).enabled(true));
    }

    @Override
    public void setServoPwmDisable(int servo) {
        ports.put(servo, port(servo).enabled(false));
    }

    @Override
    public boolean isServoPwmEnabled(int servo) {
        return port(servo).enabled();
    }

    @Override
    public void setServoType(int servo, ServoConfigurationType servoType) {}

    @Override
    public void setPulseWidth(int servo, double usPulse) {
        ports.put(servo, accepted(port(servo).pulsed(usPulse)));
    }

    @Override
    public double getPulseWidth(int servo) {
        return port(servo).pulseWidth();
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
