package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public final class NuggetHardware {
    public final DcMotorEx left;
    public final DcMotorEx right;
    public final VoltageSensor voltageSensor;
    public final Dashboard dashboard;
    public final LongSupplier nanoClock;

    private NuggetHardware(Builder wiring) {
        this.left = wiring.left;
        this.right = wiring.right;
        this.voltageSensor = wiring.voltageSensor;
        this.dashboard = wiring.dashboard;
        this.nanoClock = wiring.nanoClock;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static NuggetHardware fromHardwareMap(HardwareMap hardwareMap) {
        return builder()
                .left(hardwareMap.get(DcMotorEx.class, "leftDrive"))
                .right(hardwareMap.get(DcMotorEx.class, "rightDrive"))
                .voltageSensor(hardwareMap.voltageSensor.iterator().next())
                .dashboard(Dashboard.ftc())
                .nanoClock(System::nanoTime)
                .build();
    }

    public static final class Builder {
        private DcMotorEx left;
        private DcMotorEx right;
        private VoltageSensor voltageSensor;
        private Dashboard dashboard;
        private LongSupplier nanoClock;

        private Builder() {}

        public Builder left(DcMotorEx left) {
            this.left = left;
            return this;
        }

        public Builder right(DcMotorEx right) {
            this.right = right;
            return this;
        }

        public Builder voltageSensor(VoltageSensor voltageSensor) {
            this.voltageSensor = voltageSensor;
            return this;
        }

        public Builder dashboard(Dashboard dashboard) {
            this.dashboard = dashboard;
            return this;
        }

        public Builder nanoClock(LongSupplier nanoClock) {
            this.nanoClock = nanoClock;
            return this;
        }

        public NuggetHardware build() {
            List<String> missing = new ArrayList<>();
            named("left", left, missing);
            named("right", right, missing);
            named("voltageSensor", voltageSensor, missing);
            named("dashboard", dashboard, missing);
            named("nanoClock", nanoClock, missing);
            if (!missing.isEmpty()) {
                throw new IllegalStateException("Nugget's hardware is missing " + String.join(", ", missing));
            }
            return new NuggetHardware(this);
        }

        private static void named(String name, Object device, List<String> missing) {
            if (device == null) {
                missing.add(name);
            }
        }
    }
}
