package org.firstinspires.ftc.framework.hardware;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.function.LongSupplier;

final class SwitchedOffHardware implements Hardware {
    private final Hardware hardware;

    SwitchedOffHardware(Hardware hardware) {
        this.hardware = hardware;
    }

    @Override
    public <D> D device(Class<D> kind, String name) {
        if (!kind.isInterface()) {
            throw new IllegalStateException("a switched-off part cannot be handed the " + kind.getSimpleName() + " "
                    + name + ": only a device the SDK names by an interface, such as DcMotorEx, can be switched off");
        }
        return inert(kind, name);
    }

    @Override
    public LazyImu imu() {
        IMU imu = inert(IMU.class, "imu");
        return () -> imu;
    }

    @Override
    public VoltageSensor voltageSensor() {
        return inert(VoltageSensor.class, "voltageSensor");
    }

    @Override
    public Dashboard dashboard() {
        return hardware.dashboard();
    }

    @Override
    public LongSupplier nanoClock() {
        return hardware.nanoClock();
    }

    @Override
    public Hardware switchedOff() {
        return this;
    }

    private static <D> D inert(Class<D> kind, String name) {
        String description = "switched-off " + kind.getSimpleName() + " " + name;
        return kind.cast(Proxy.newProxyInstance(
                kind.getClassLoader(), new Class<?>[] {kind}, (proxy, method, args) -> {
                    if (method.getDeclaringClass() == Object.class) {
                        return objectMethod(proxy, method, args, description);
                    }
                    return nothing(method.getReturnType(), description);
                }));
    }

    private static Object objectMethod(Object proxy, Method method, Object[] args, String description) {
        switch (method.getName()) {
            case "equals":
                return proxy == args[0];
            case "hashCode":
                return System.identityHashCode(proxy);
            default:
                return description;
        }
    }

    private static Object nothing(Class<?> kind, String description) {
        if (kind == boolean.class) {
            return false;
        }
        if (kind == int.class) {
            return 0;
        }
        if (kind == long.class) {
            return 0L;
        }
        if (kind == double.class) {
            return 0.0;
        }
        if (kind == float.class) {
            return 0f;
        }
        if (kind == short.class) {
            return (short) 0;
        }
        if (kind == byte.class) {
            return (byte) 0;
        }
        if (kind == char.class) {
            return (char) 0;
        }
        if (kind == String.class) {
            return description;
        }
        return null;
    }
}
