package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.Objects;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Every subsystem, built afresh for one run of an op mode, plus what they all share: the alliance
 * the run plays for, the gamepads, the telemetry and the dashboard. A subsystem reaches all of
 * that through the robot it was {@link #add}ed to instead of being handed each piece.
 */
public final class Robot implements Loopable {
    public final Alliance alliance;
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;
    public final Telemetry telemetry;
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
     * Shared by the {@link Localizer}, which moves the pose on through it, and {@link Nav}, which
     * builds paths with it.
     */
    public final MecanumDrive mecanumDrive;

    public final Turntable turntable;
    public final Brain brain;
    public final Plans plans;
    private final LoopGroup loop = new LoopGroup();

    /** A robot nobody is driving: its gamepads stay idle. For tests of what needs no driver. */
    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry) {
        this(hardware, alliance, telemetry, new Gamepad(), new Gamepad());
    }

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.alliance = alliance;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.dashboard = hardware.dashboard;
        this.clock = Objects.requireNonNull(hardware.clock, "the hardware has no clock");
        launcher = new Launcher(hardware.launcher, hardware.topGate, hardware.bottomGate);
        intake = new Intake(hardware.intake);
        drive = new Drive(hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack);
        camera = new Camera(hardware.aprilTags, clock);
        // The dead wheels are read through the rightBack (parallel) and leftFront (perpendicular)
        // encoder ports, which is how they are wired: the same motors the drive turns.
        localizer = new Localizer(hardware.rightBack, hardware.leftFront, hardware.imu, new Pose2d(0, 0, 0), clock);
        mecanumDrive = new MecanumDrive(
                hardware.leftFront,
                hardware.leftBack,
                hardware.rightBack,
                hardware.rightFront,
                hardware.imu,
                hardware.voltageSensor,
                localizer.deadWheels(),
                clock);
        nav = new Nav(mecanumDrive, localizer, alliance);
        turntable = new Turntable(hardware.turnTable);
        brain = new Brain();
        plans = new Plans();
        // Registration order is loop order. The localizer goes first: where the robot is is the
        // first fact of a tick, and everything that reads the pose during the tick reads the one
        // it settled on. Brain goes after the subsystems it coordinates because it reads Camera
        // and Nav from this tick and sets the Turntable target.
        for (SubSystem subSystem : List.of(localizer, launcher, intake, drive, camera, nav, turntable, brain, plans)) {
            add(subSystem);
        }
    }

    /**
     * Registers something to tick after everything registered so far. A subsystem also learns
     * this robot and is initialised.
     */
    public <T extends Loopable> T add(T loopable) {
        if (loopable instanceof SubSystem) {
            ((SubSystem) loopable).attach(this);
        }
        loop.add(loopable);
        if (loopable instanceof SubSystem) {
            ((SubSystem) loopable).init();
        }
        return loopable;
    }

    /** Everything this robot ticks, in order. */
    public List<Loopable> loopOrder() {
        return loop.members();
    }

    @Override
    public void loop() {
        loop.loop();
    }
}
