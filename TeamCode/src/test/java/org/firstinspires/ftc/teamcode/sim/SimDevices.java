package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServoController;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.HubMounting;
import org.firstinspires.ftc.teamcode.simcore.MotorType;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.ServoSignal;

public final class SimDevices {
    public static final double BATTERY_VOLTS = 12.5;

    public final FakeImu imu;
    public final FakeVoltageSensor voltageSensor = new FakeVoltageSensor(BATTERY_VOLTS);
    public final FakeDashboard dashboard = new FakeDashboard();
    public final FakeServoController servos = new FakeServoController();
    public CRServoImplEx spinner;
    public ServoImplEx ramp;

    private final Map<String, FakeDcMotorEx> motors = new LinkedHashMap<>();
    private final Map<String, EncoderReads> encoders = new LinkedHashMap<>();
    private final Map<String, MotorType> types = new LinkedHashMap<>();
    private final SimHardwareMap hardwareMap = new SimHardwareMap();
    private long nanos = 0;

    public SimDevices() {
        this(RobotDescription.reginalds());
    }

    public SimDevices(RobotDescription description) {
        imu = FakeImu.mounted(sdkMounting(description.hub()));
        int servoPort = 0;
        for (Configuration.Device device : description.configuration().devices()) {
            String name = device.name();
            if (device instanceof Configuration.Motor motor) {
                FakeDcMotorEx fake = new FakeDcMotorEx(sdkType(motor.type()));
                motors.put(name, fake);
                encoders.put(name, motor.encoder());
                types.put(name, motor.type());
                hardwareMap.dcMotor.put(name, fake);
            } else if (device instanceof Configuration.Imu) {
                hardwareMap.put(name, imu);
            } else if (device instanceof Configuration.ContinuousServo) {
                spinner = new CRServoImplEx(servos, servoPort++, DcMotorSimple.Direction.FORWARD, null);
                hardwareMap.crservo.put(name, spinner);
            } else if (device instanceof Configuration.Servo) {
                ramp = new ServoImplEx(servos, servoPort++, null);
                hardwareMap.servo.put(name, ramp);
            } else if (device instanceof Configuration.VoltageSensor) {
                hardwareMap.voltageSensor.put(name, voltageSensor);
            }
        }
    }

    public ServoSignal spinnerSignal() {
        return spinner == null ? ServoSignal.none() : signalTo(spinner.getPortNumber());
    }

    public ServoSignal rampSignal() {
        return ramp == null ? ServoSignal.none() : signalTo(ramp.getPortNumber());
    }

    private ServoSignal signalTo(int port) {
        PwmControl.PwmRange range = servos.getServoPwmRange(port);
        return Valid.value(
                ServoSignal.of(servos.driven(port), servos.position(port), range.usPulseLower, range.usPulseUpper));
    }

    static RevHubOrientationOnRobot sdkMounting(HubMounting hub) {
        return new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.valueOf(hub.logo().name()),
                RevHubOrientationOnRobot.UsbFacingDirection.valueOf(hub.usb().name()));
    }

    static MotorConfigurationType sdkType(MotorType type) {
        Class<?> declaring = sdkClassOf(type);
        MotorConfigurationType sdk = new MotorConfigurationType();
        sdk.processAnnotation(
                declaring.getAnnotation(com.qualcomm.robotcore.hardware.configuration.annotations.MotorType.class));
        try {
            Field name = UserConfigurationType.class.getDeclaredField("name");
            name.setAccessible(true);
            name.set(sdk, type.label());
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("the SDK's motor type no longer keeps its name where it did", e);
        }
        return sdk;
    }

    private static Class<?> sdkClassOf(MotorType type) {
        switch (type) {
            case ULTRAPLANETARY_HD_HEX:
                return RevRoboticsUltraPlanetaryHdHexMotor.class;
        }
        throw new IllegalArgumentException("the simulator knows no SDK class for " + type.label());
    }

    public FakeDcMotorEx motor(String name) {
        FakeDcMotorEx motor = motors.get(name);
        if (motor == null) {
            throw new IllegalArgumentException(
                    "no motor is configured as " + name + "; the motors are " + String.join(", ", motors.keySet()));
        }
        return motor;
    }

    public List<String> motorNames() {
        return List.copyOf(motors.keySet());
    }

    public MotorType typeOf(String name) {
        motor(name);
        return types.get(name);
    }

    public EncoderReads encoderOf(String name) {
        motor(name);
        return encoders.get(name);
    }

    public HardwareMap hardwareMap() {
        return hardwareMap;
    }

    public Hardware hardware() {
        return onTheSimulatorsClockAndDashboard(Hardware.fromHardwareMap(hardwareMap));
    }

    private Hardware onTheSimulatorsClockAndDashboard(Hardware found) {
        Hardware.Builder builder = Hardware.builder();
        for (Method setter : builder.getClass().getMethods()) {
            if (setter.getDeclaringClass() != builder.getClass() || setter.getParameterCount() != 1) {
                continue;
            }
            try {
                setter.invoke(builder, handed(found, setter.getName()));
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(
                        "the simulator could not hand the robot's hardware its " + setter.getName(), e);
            } catch (InvocationTargetException e) {
                throw new IllegalStateException(
                        "the robot's hardware refused the simulator's " + setter.getName(), e.getCause());
            }
        }
        return builder.build();
    }

    private Object handed(Hardware found, String name) throws IllegalAccessException {
        if (name.equals("dashboard")) {
            return dashboard;
        }
        if (name.equals("nanoClock")) {
            return (LongSupplier) this::nanoTime;
        }
        try {
            return Hardware.class.getField(name).get(found);
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException(
                    "the robot's hardware asks for " + name
                            + " and keeps no field of that name for the simulator to hand on",
                    e);
        }
    }

    public long nanoTime() {
        return nanos;
    }

    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
