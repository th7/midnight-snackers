package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public final class Hardware {
    public final DcMotorEx launcher;
    public final Servo topGate;
    public final Servo bottomGate;
    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx turnTable;
    public final DcMotorEx intake;

    public final LazyImu imu;

    public final VoltageSensor voltageSensor;
    public final Supplier<List<AprilTagDetection>> aprilTags;
    public final Dashboard dashboard;

    public final LongSupplier nanoClock;

    private Hardware(Builder wiring) {
        this.launcher = wiring.launcher;
        this.topGate = wiring.topGate;
        this.bottomGate = wiring.bottomGate;
        this.leftFront = wiring.leftFront;
        this.rightFront = wiring.rightFront;
        this.leftBack = wiring.leftBack;
        this.rightBack = wiring.rightBack;
        this.turnTable = wiring.turnTable;
        this.intake = wiring.intake;
        this.imu = wiring.imu;
        this.voltageSensor = wiring.voltageSensor;
        this.aprilTags = wiring.aprilTags;
        this.dashboard = wiring.dashboard;
        this.nanoClock = wiring.nanoClock;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static Hardware fromHardwareMap(HardwareMap hardwareMap) {
        return builder()
                .launcher(hardwareMap.get(DcMotorEx.class, "launcher"))
                .topGate(hardwareMap.get(Servo.class, "topGate"))
                .bottomGate(hardwareMap.get(Servo.class, "bottomGate"))
                .leftFront(hardwareMap.get(DcMotorEx.class, "leftFront"))
                .rightFront(hardwareMap.get(DcMotorEx.class, "rightFront"))
                .leftBack(hardwareMap.get(DcMotorEx.class, "leftBack"))
                .rightBack(hardwareMap.get(DcMotorEx.class, "rightBack"))
                .turnTable(hardwareMap.get(DcMotorEx.class, "turnTable"))
                .intake(hardwareMap.get(DcMotorEx.class, "intake"))
                .imu(new LazyHardwareMapImu(
                        hardwareMap,
                        "imu",
                        new RevHubOrientationOnRobot(
                                MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection)))
                .voltageSensor(hardwareMap.voltageSensor.iterator().next())
                .aprilTags(AprilTagWebcam.detections(hardwareMap))
                .dashboard(Dashboard.ftc())
                .nanoClock(System::nanoTime)
                .build();
    }

    public static final class Builder {
        private DcMotorEx launcher;
        private Servo topGate;
        private Servo bottomGate;
        private DcMotorEx leftFront;
        private DcMotorEx rightFront;
        private DcMotorEx leftBack;
        private DcMotorEx rightBack;
        private DcMotorEx turnTable;
        private DcMotorEx intake;
        private LazyImu imu;
        private VoltageSensor voltageSensor;
        private Supplier<List<AprilTagDetection>> aprilTags;
        private Dashboard dashboard;
        private LongSupplier nanoClock;

        private Builder() {}

        public Builder launcher(DcMotorEx launcher) {
            this.launcher = launcher;
            return this;
        }

        public Builder topGate(Servo topGate) {
            this.topGate = topGate;
            return this;
        }

        public Builder bottomGate(Servo bottomGate) {
            this.bottomGate = bottomGate;
            return this;
        }

        public Builder leftFront(DcMotorEx leftFront) {
            this.leftFront = leftFront;
            return this;
        }

        public Builder rightFront(DcMotorEx rightFront) {
            this.rightFront = rightFront;
            return this;
        }

        public Builder leftBack(DcMotorEx leftBack) {
            this.leftBack = leftBack;
            return this;
        }

        public Builder rightBack(DcMotorEx rightBack) {
            this.rightBack = rightBack;
            return this;
        }

        public Builder turnTable(DcMotorEx turnTable) {
            this.turnTable = turnTable;
            return this;
        }

        public Builder intake(DcMotorEx intake) {
            this.intake = intake;
            return this;
        }

        public Builder imu(LazyImu imu) {
            this.imu = imu;
            return this;
        }

        public Builder voltageSensor(VoltageSensor voltageSensor) {
            this.voltageSensor = voltageSensor;
            return this;
        }

        public Builder aprilTags(Supplier<List<AprilTagDetection>> aprilTags) {
            this.aprilTags = aprilTags;
            return this;
        }

        public Builder dashboard(Dashboard dashboard) {
            this.dashboard = dashboard;
            return this;
        }

        public Builder nanoClock(LongSupplier nanoClock) {
            this.nanoClock = nanoClock;
            return this;
        }

        public Hardware build() {
            List<String> missing = new ArrayList<>();
            named("launcher", launcher, missing);
            named("topGate", topGate, missing);
            named("bottomGate", bottomGate, missing);
            named("leftFront", leftFront, missing);
            named("rightFront", rightFront, missing);
            named("leftBack", leftBack, missing);
            named("rightBack", rightBack, missing);
            named("turnTable", turnTable, missing);
            named("intake", intake, missing);
            named("imu", imu, missing);
            named("voltageSensor", voltageSensor, missing);
            named("aprilTags", aprilTags, missing);
            named("dashboard", dashboard, missing);
            named("nanoClock", nanoClock, missing);
            if (!missing.isEmpty()) {
                throw new IllegalStateException("the hardware has no " + String.join(", ", missing));
            }
            return new Hardware(this);
        }

        private static void named(String name, Object device, List<String> missing) {
            if (device == null) {
                missing.add(name);
            }
        }
    }
}
