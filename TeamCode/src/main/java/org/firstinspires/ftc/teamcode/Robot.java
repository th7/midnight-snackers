package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Channels;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Every subsystem, built afresh for one run of an op mode, plus what they all share: the alliance
 * the run plays for, the gamepads, the telemetry and the dashboard. A subsystem is handed what it
 * needs when it is built and reaches back for nothing, so the order below is the order they depend
 * on each other in, and javac says so.
 */
public final class Robot implements Loopable {
    public final Alliance alliance;
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;
    public final Telemetry telemetry;
    /** The driver station's telemetry by channel: each subsystem prints to its own. */
    public final Channels channels;

    public final Dashboard dashboard;
    /** The hardware's clock, in nanoseconds: what every timer here runs on. */
    public final LongSupplier clock;

    public final Launcher launcher;
    public final Intake intake;
    public final Drive drive;
    public final Camera camera;
    public final Localizer localizer;
    public final Nav nav;
    /**
     * Road Runner's drive: the trajectories it builds and follows, and the pose history it draws.
     * Held by the {@link Drive}, which builds paths with it and follows them, and given the
     * {@link Localizer} to read the pose from. Nav says where; the drive says how, and
     * {@code NavTest.navDoesNotHoldTheDriveThatBuildsPaths} keeps it that way.
     */
    public final MecanumDrive mecanumDrive;
    /** The four wheels: the one place a power reaches a drive motor. */
    public final Wheels wheels;

    public final Turntable turntable;
    public final Brain brain;
    public final Plans plans;
    /** Everything this robot ticks, in the order it ticks them. */
    private final List<Loopable> loopOrder;

    /** A robot nobody is driving: its gamepads stay idle. For tests of what needs no driver. */
    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry) {
        this(hardware, alliance, telemetry, new Gamepad(), new Gamepad());
    }

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.alliance = alliance;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.channels = new Channels(telemetry);
        this.dashboard = hardware.dashboard;
        // Every device is there, or the hardware would not have been built.
        this.clock = hardware.clock;
        // Each is handed what it needs, so the order here is the order they depend on each other
        // in, and javac says so: a subsystem built before one it is given does not compile.
        launcher = new Launcher(
                hardware.launcher, hardware.topGate, hardware.bottomGate, clock, channels.channel(Launcher.CHANNEL));
        intake = new Intake(hardware.intake);
        camera = new Camera(hardware.aprilTags, clock, channels.channel(Camera.CHANNEL));
        turntable = new Turntable(hardware.turnTable, channels.channel(Turntable.CHANNEL));
        // One Wheels, shared: the drive asks it to turn the robot by hand or toward a pose, and
        // Road Runner's drive asks it while following a trajectory. Nothing else may.
        wheels = new Wheels(hardware.leftFront, hardware.leftBack, hardware.rightBack, hardware.rightFront);
        // The dead wheels are read through the rightBack (parallel) and leftFront (perpendicular)
        // encoder ports, which is how they are wired: the same motors the drive turns.
        localizer = new Localizer(
                hardware.rightBack,
                hardware.leftFront,
                hardware.imu,
                new Pose2d(0, 0, 0),
                clock,
                channels.channel(Localizer.CHANNEL));
        mecanumDrive = new MecanumDrive(wheels, hardware.imu, hardware.voltageSensor, localizer, clock);
        drive = new Drive(wheels, mecanumDrive, localizer, dashboard, channels.channel(Drive.CHANNEL));
        nav = new Nav(localizer, alliance);
        brain = new Brain(drive, launcher, camera, nav, turntable, alliance, channels.channel(Brain.CHANNEL));
        plans = new Plans(drive, nav, launcher, clock);
        // This list is the loop order. The localizer goes first: where the robot is is the first
        // fact of a tick, and everything that reads the pose during the tick reads the one it
        // settled on. Brain goes after the subsystems it coordinates because it reads Camera and
        // Nav from this tick and aims the Turntable.
        this.loopOrder = List.of(localizer, launcher, intake, drive, camera, nav, turntable, brain, plans);
    }

    /** Everything this robot ticks, in order. Anything else belongs to whoever owns it. */
    public List<Loopable> loopOrder() {
        return loopOrder;
    }

    @Override
    public void loop() {
        for (Loopable subSystem : loopOrder) {
            subSystem.loop();
        }
    }
}
