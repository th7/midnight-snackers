package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

public final class NuggetHardware {
    public final DcMotorEx left;
    public final DcMotorEx right;

    private NuggetHardware(Builder wiring) {
        this.left = wiring.left;
        this.right = wiring.right;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static NuggetHardware fromHardwareMap(HardwareMap hardwareMap) {
        return builder()
                .left(hardwareMap.get(DcMotorEx.class, "leftDrive"))
                .right(hardwareMap.get(DcMotorEx.class, "rightDrive"))
                .build();
    }

    public static final class Builder {
        private DcMotorEx left;
        private DcMotorEx right;

        private Builder() {}

        public Builder left(DcMotorEx left) {
            this.left = left;
            return this;
        }

        public Builder right(DcMotorEx right) {
            this.right = right;
            return this;
        }

        public NuggetHardware build() {
            List<String> missing = new ArrayList<>();
            if (left == null) {
                missing.add("left");
            }
            if (right == null) {
                missing.add("right");
            }
            if (!missing.isEmpty()) {
                throw new IllegalStateException("Nugget's hardware is missing " + String.join(", ", missing));
            }
            return new NuggetHardware(this);
        }
    }
}
