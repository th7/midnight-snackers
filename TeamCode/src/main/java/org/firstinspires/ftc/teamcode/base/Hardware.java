package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Supplier;

/**
 * Everything the op modes touch outside their own code: the configured devices, the camera's
 * detections, and the dashboard. {@link #fromHardwareMap} is the real robot; tests and the
 * simulator build one from fakes.
 */
public final class Hardware {
    public final DcMotorEx launcher;
    public final Servo topGate;
    public final Servo bottomGate;
    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx turnTable;
    /**
     * Initialized with the hub orientation from {@link MecanumDrive.Params} on first use.
     */
    public final LazyImu imu;
    public final VoltageSensor voltageSensor;
    public final Supplier<List<AprilTagDetection>> aprilTags;
    public final Dashboard dashboard;

    public Hardware(DcMotorEx launcher, Servo topGate, Servo bottomGate,
                    DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack,
                    DcMotorEx turnTable, LazyImu imu, VoltageSensor voltageSensor,
                    Supplier<List<AprilTagDetection>> aprilTags, Dashboard dashboard) {
        this.launcher = launcher;
        this.topGate = topGate;
        this.bottomGate = bottomGate;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.turnTable = turnTable;
        this.imu = imu;
        this.voltageSensor = voltageSensor;
        this.aprilTags = aprilTags;
        this.dashboard = dashboard;
    }

    /**
     * The devices from the robot configuration. Only valid on the robot controller.
     */
    public static Hardware fromHardwareMap(HardwareMap hardwareMap) {
        return new Hardware(
                hardwareMap.get(DcMotorEx.class, "launcher"),
                hardwareMap.get(Servo.class, "topGate"),
                hardwareMap.get(Servo.class, "bottomGate"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                hardwareMap.get(DcMotorEx.class, "turnTable"),
                new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                        MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection)),
                hardwareMap.voltageSensor.iterator().next(),
                AprilTagWebcam.detections(hardwareMap),
                Dashboard.ftc());
    }
}
