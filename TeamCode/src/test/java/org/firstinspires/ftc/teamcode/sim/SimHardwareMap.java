package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

final class SimHardwareMap extends HardwareMap {
    SimHardwareMap() {
        super(null, null);
    }

    @Override
    public <T> T tryGet(Class<? extends T> kind, String name) {
        synchronized (lock) {
            List<HardwareDevice> named = allDevicesMap.get(name.trim());
            if (named == null) {
                return null;
            }
            for (HardwareDevice device : named) {
                if (kind.isInstance(device)) {
                    return kind.cast(device);
                }
            }
            return null;
        }
    }
}
