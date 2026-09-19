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

public final class Robot implements Loopable {
    public final Alliance alliance;
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;
    public final Telemetry telemetry;

    public final Channels channels;

    public final Dashboard dashboard;

    public final LongSupplier nanoClock;

    public final Launcher launcher;
    public final Intake intake;
    public final Drive drive;
    public final Camera camera;
    public final Localizer localizer;
    public final Nav nav;

    public final MecanumDrive mecanumDrive;

    public final Wheels wheels;

    public final Turntable turntable;
    public final Brain brain;
    public final Plans plans;

    private final List<Loopable> loopOrder;

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

        this.nanoClock = hardware.nanoClock;

        launcher = new Launcher(
                hardware.launcher,
                hardware.topGate,
                hardware.bottomGate,
                nanoClock,
                channels.channel(Launcher.CHANNEL));
        intake = new Intake(hardware.intake);
        camera = new Camera(hardware.aprilTags, nanoClock, channels.channel(Camera.CHANNEL));
        turntable = new Turntable(hardware.turnTable, channels.channel(Turntable.CHANNEL));

        wheels = new Wheels(hardware.leftFront, hardware.leftBack, hardware.rightBack, hardware.rightFront);

        localizer = new Localizer(
                hardware.rightBack,
                hardware.leftFront,
                hardware.imu,
                new Pose2d(0, 0, 0),
                nanoClock,
                channels.channel(Localizer.CHANNEL));
        mecanumDrive = new MecanumDrive(wheels, hardware.imu, hardware.voltageSensor, localizer, nanoClock);
        drive = new Drive(wheels, mecanumDrive, localizer, dashboard, channels.channel(Drive.CHANNEL));
        nav = new Nav(localizer, alliance);
        brain = new Brain(drive, launcher, camera, nav, turntable, alliance, channels.channel(Brain.CHANNEL));
        plans = new Plans(drive, nav, launcher, nanoClock);

        this.loopOrder = List.of(localizer, launcher, intake, drive, camera, nav, turntable, brain, plans);
    }

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
