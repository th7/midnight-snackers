package org.firstinspires.ftc.reginald;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.reginald.hardware.Wheels;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Channels;
import org.firstinspires.ftc.teamcode.base.LoopOrder;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public final class Robot implements Loopable {
    public final Alliance alliance;
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;
    public final Telemetry telemetry;

    public final Channels channels;

    public final Dashboard dashboard;

    public final LongSupplier nanoClock;
    public final Intake intake;

    // public final Launcher launcher;
    public final Drive drive;
    // public final Camera camera;
    public final Localizer localizer;
    public final Nav nav;

    public final MecanumDrive mecanumDrive;

    public final Wheels wheels;

    // public final Turntable turntable;
    public final Brain brain;
    public final Plans plans;

    private final LoopOrder loopOrder = new LoopOrder();

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry) {
        this(hardware, alliance, telemetry, new Gamepad(), new Gamepad());
    }

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.alliance = alliance;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.channels = new Channels(telemetry);
        this.dashboard = hardware.dashboard();

        this.nanoClock = hardware.nanoClock();

        localizer = new Localizer(
                loopOrder,
                hardware.motor("rightBack"),
                hardware.motor("leftFront"),
                hardware.imu(),
                new Pose2d(0, 0, 0),
                nanoClock,
                channels.channel(Localizer.CHANNEL));

        Hardware intakeHardware = hardware;

        /* launcher = new Launcher(
          loopOrder,
          hardware.motor("launcher"),
          hardware.servo("topGate"),
          hardware.servo("bottomGate"),
          nanoClock,
        channels.channel(Launcher.CHANNEL)); */
        intake = new Intake(loopOrder, intakeHardware.motor("intake"));
        // camera = new Camera(loopOrder, hardware.aprilTags(), nanoClock, channels.channel(Camera.CHANNEL));
        //  turntable = new Turntable(loopOrder, hardware.motor("turnTable"), channels.channel(Turntable.CHANNEL));

        wheels = new Wheels(
                hardware.motor("leftFront"),
                hardware.motor("leftBack"),
                hardware.motor("rightBack"),
                hardware.motor("rightFront"));

        mecanumDrive = new MecanumDrive(wheels, hardware.imu(), hardware.voltageSensor(), localizer, nanoClock);
        drive = new Drive(loopOrder, wheels, mecanumDrive, localizer, dashboard, channels.channel(Drive.CHANNEL));
        nav = new Nav(loopOrder, localizer, alliance);
        brain = new Brain(loopOrder, drive, nav, alliance, channels.channel(Brain.CHANNEL));
        plans = new Plans(loopOrder, drive, nav, nanoClock, intake);
    }

    @Override
    public void loop() {
        loopOrder.loop();
    }
}
