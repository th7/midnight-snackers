package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;
import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.framework.hardware.Dashboard;
import org.firstinspires.ftc.framework.hardware.Hardware;
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
        for (Configuration.Committed committed : description.configuration().committed()) {
            Configuration.Device device = committed.device();
            String name = device.name();
            if (device instanceof Configuration.Motor motor) {
                FakeDcMotorEx fake = new FakeDcMotorEx(sdkType(motor.type()));
                motors.put(committed.as(), fake);
                encoders.put(committed.as(), motor.encoder());
                types.put(committed.as(), motor.type());
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
        return spinner == null ? ServoSignal.none() : servos.signal(spinner.getPortNumber());
    }

    public ServoSignal rampSignal() {
        return ramp == null ? ServoSignal.none() : servos.signal(ramp.getPortNumber());
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

    public FakeDcMotorEx motor(String committedAs) {
        FakeDcMotorEx motor = motors.get(committedAs);
        if (motor == null) {
            throw new IllegalArgumentException("no motor is committed as " + committedAs
                    + "; the motors are committed as " + String.join(", ", motors.keySet()));
        }
        return motor;
    }

    public List<String> motorsCommittedAs() {
        return List.copyOf(motors.keySet());
    }

    public MotorType typeOf(String committedAs) {
        motor(committedAs);
        return types.get(committedAs);
    }

    public EncoderReads encoderOf(String committedAs) {
        motor(committedAs);
        return encoders.get(committedAs);
    }

    public HardwareMap hardwareMap() {
        return hardwareMap;
    }

    public Hardware hardware() {
        Hardware found = Hardware.fromHardwareMap(hardwareMap);
        return new Hardware() {
            @Override
            public <D> D device(Class<D> kind, String name) {
                return found.device(kind, name);
            }

            @Override
            public LazyImu imu() {
                return found.imu();
            }

            @Override
            public VoltageSensor voltageSensor() {
                return found.voltageSensor();
            }

            @Override
            public Dashboard dashboard() {
                return dashboard;
            }

            @Override
            public LongSupplier nanoClock() {
                return SimDevices.this::nanoTime;
            }
        };
    }

    public long nanoTime() {
        return nanos;
    }

    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
