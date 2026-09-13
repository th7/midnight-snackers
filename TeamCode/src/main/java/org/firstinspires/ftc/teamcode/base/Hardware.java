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
 * simulator fill one in from fakes, by name.
 */
public final class Hardware {
    public DcMotorEx launcher;
    public Servo topGate;
    public Servo bottomGate;
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx turnTable;
    /**
     * Initialized with the hub orientation from {@link MecanumDrive.Params} on first use.
     */
    public LazyImu imu;
    public VoltageSensor voltageSensor;
    public Supplier<List<AprilTagDetection>> aprilTags;
    public Dashboard dashboard;

    /**
     * The devices from the robot configuration. Only valid on the robot controller.
     */
    public static Hardware fromHardwareMap(HardwareMap hardwareMap) {
        Hardware hardware = new Hardware();
        hardware.launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        hardware.topGate = hardwareMap.get(Servo.class, "topGate");
        hardware.bottomGate = hardwareMap.get(Servo.class, "bottomGate");
        hardware.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        hardware.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        hardware.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        hardware.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        hardware.turnTable = hardwareMap.get(DcMotorEx.class, "turnTable");
        hardware.imu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
        hardware.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hardware.aprilTags = AprilTagWebcam.detections(hardwareMap);
        hardware.dashboard = Dashboard.ftc();
        return hardware;
    }
}
