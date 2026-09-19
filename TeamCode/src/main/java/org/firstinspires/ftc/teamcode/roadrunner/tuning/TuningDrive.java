package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public final class TuningDrive {
    public final Localizer localizer;

    public final MecanumDrive drive;

    private TuningDrive(Localizer localizer, MecanumDrive drive) {
        this.localizer = localizer;
        this.drive = drive;
    }

    public static TuningDrive from(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        LazyImu lazyImu = new LazyHardwareMapImu(
                hardwareMap,
                "imu",
                new RevHubOrientationOnRobot(
                        MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
        Wheels wheels = new Wheels(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                hardwareMap.get(DcMotorEx.class, "rightFront"));

        Localizer localizer = new Localizer(
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                lazyImu,
                pose,
                System::nanoTime,
                Prints.NOWHERE);
        MecanumDrive drive = new MecanumDrive(
                wheels, lazyImu, hardwareMap.voltageSensor.iterator().next(), localizer, System::nanoTime);
        return new TuningDrive(localizer, drive);
    }
}
