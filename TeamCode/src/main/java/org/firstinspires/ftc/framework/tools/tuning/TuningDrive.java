package org.firstinspires.ftc.framework.tools.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.framework.drive.Localizer;
import org.firstinspires.ftc.framework.drive.MecanumDrive;
import org.firstinspires.ftc.framework.hardware.Hardware;
import org.firstinspires.ftc.framework.hardware.Wheels;
import org.firstinspires.ftc.framework.robot.Prints;

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

        Hardware hardware = Hardware.fromHardwareMap(hardwareMap);
        Wheels wheels = new Wheels(hardware);
        Localizer localizer = new Localizer(hardware, Prints.NOWHERE);
        localizer.setPose(pose);
        MecanumDrive drive = new MecanumDrive(wheels, hardware, localizer);
        return new TuningDrive(localizer, drive);
    }
}
