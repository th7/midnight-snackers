package org.firstinspires.ftc.reginald.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.reginald.Localizer;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.reginald.hardware.Wheels;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.base.LoopOrder;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;

public final class TuningDrive implements Loopable {
    public final Localizer localizer;

    public final MecanumDrive drive;

    private final LoopOrder loopOrder;

    private TuningDrive(LoopOrder loopOrder, Localizer localizer, MecanumDrive drive) {
        this.loopOrder = loopOrder;
        this.localizer = localizer;
        this.drive = drive;
    }

    @Override
    public void loop() {
        loopOrder.loop();
    }

    public static TuningDrive from(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Hardware hardware = Hardware.fromHardwareMap(hardwareMap);
        Wheels wheels = new Wheels(
                hardware.motor("leftFront"),
                hardware.motor("leftBack"),
                hardware.motor("rightBack"),
                hardware.motor("rightFront"));

        LoopOrder loopOrder = new LoopOrder();
        Localizer localizer = new Localizer(
                loopOrder,
                hardware.motor("rightBack"),
                hardware.motor("leftFront"),
                hardware.imu(),
                pose,
                hardware.nanoClock(),
                Prints.NOWHERE);
        MecanumDrive drive = new MecanumDrive(
                wheels, hardware.imu(), hardware.voltageSensor(), localizer, hardware.nanoClock());
        return new TuningDrive(loopOrder, localizer, drive);
    }
}
