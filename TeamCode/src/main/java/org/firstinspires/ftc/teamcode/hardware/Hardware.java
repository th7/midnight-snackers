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

/**
 * Everything the op modes touch outside their own code: the configured devices, the camera's
 * detections, the dashboard, and the clock. {@link #fromHardwareMap} is the real robot; tests and
 * the simulator fill one in from fakes.
 *
 * <p>It is wired by hand on both sides of the seam and nothing but this class can say the two
 * agree, so a hardware is made whole or not at all: {@link Builder#build()} refuses one that is
 * missing a device and names what is missing. A device added here that an adapter has not kept up
 * with then fails where that adapter is written, rather than as a null inside whichever subsystem
 * reaches for it first -- which, on the robot, is in the middle of a match.
 *
 * <p>The devices are named as they are set, not counted out in order: nine of them are motors, so
 * a positional list would let a wheel be quietly swapped for its neighbour.
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
    public final DcMotorEx intake;
    /**
     * Initialized with the hub orientation from {@link MecanumDrive.Params} on first use.
     */
    public final LazyImu imu;

    public final VoltageSensor voltageSensor;
    public final Supplier<List<AprilTagDetection>> aprilTags;
    public final Dashboard dashboard;
    /**
     * The time, in nanoseconds from an arbitrary origin, as {@link System#nanoTime()} gives it. Every
     * timer in the robot code reads this and nothing else, so a simulation can own time: run faster
     * than real time, and the same way every time.
     */
    public final LongSupplier clock;

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
        this.clock = wiring.clock;
    }

    /** Somewhere to name every device, for whoever is driving the robot code. */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * The devices from the robot configuration. Only valid on the robot controller.
     */
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
                .clock(System::nanoTime)
                .build();
    }

    /**
     * Names every device a hardware is made of. {@link #build()} refuses one that is missing any,
     * so this is the one place that knows what a whole hardware is.
     */
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
        private LongSupplier clock;

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

        public Builder clock(LongSupplier clock) {
            this.clock = clock;
            return this;
        }

        /**
         * The hardware, once every device has been named.
         *
         * @throws IllegalStateException naming every device that was not
         */
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
            named("clock", clock, missing);
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
