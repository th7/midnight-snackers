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

/**
 * A drive and a localizer built from the robot configuration alone, for the tuning op modes.
 *
 * <p>They run on the robot and nowhere else, and each is a {@code LinearOpMode} that builds what
 * it needs rather than being handed a robot. So the wiring the robot does in its constructor --
 * the wheels, the dead wheels, the clock -- is done here instead, once, rather than by three op
 * modes separately or by the drive itself, which is handed these things everywhere else.
 */
public final class TuningDrive {
    /** The localizer the last {@link #from} built, which the tuning op modes tick themselves. */
    public final Localizer localizer;

    public final MecanumDrive drive;

    private TuningDrive(Localizer localizer, MecanumDrive drive) {
        this.localizer = localizer;
        this.drive = drive;
    }

    /** Everything a tuning op mode drives, from the configuration, starting at {@code pose}. */
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
        // The dead wheels are read through the rightBack (parallel) and leftFront (perpendicular)
        // encoder ports, which is how they are wired.
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
