package org.firstinspires.ftc.reginald.hardware;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public interface Hardware {
    DcMotorEx motor(String name);

    Servo servo(String name);

    LazyImu imu();

    VoltageSensor voltageSensor();

    Dashboard dashboard();

    LongSupplier nanoClock();

    default Hardware switchedOff() {
        return new SwitchedOffHardware(this);
    }

    static Hardware fromHardwareMap(HardwareMap hardwareMap) {
        LazyImu imu = new LazyHardwareMapImu(
                hardwareMap,
                "imu",
                new RevHubOrientationOnRobot(
                        MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
        return new Hardware() {
            @Override
            public DcMotorEx motor(String name) {
                return hardwareMap.get(DcMotorEx.class, name);
            }

            @Override
            public Servo servo(String name) {
                return hardwareMap.get(Servo.class, name);
            }

            @Override
            public LazyImu imu() {
                return imu;
            }

            @Override
            public VoltageSensor voltageSensor() {
                return hardwareMap.voltageSensor.iterator().next();
            }

            @Override
            public Dashboard dashboard() {
                return Dashboard.ftc();
            }

            @Override
            public LongSupplier nanoClock() {
                return System::nanoTime;
            }
        };
    }
}
