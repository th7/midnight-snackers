package org.firstinspires.ftc.reginald.opmode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.reginald.Brain;
import org.firstinspires.ftc.reginald.Camera;
import org.firstinspires.ftc.reginald.Drive;
import org.firstinspires.ftc.reginald.Launcher;
import org.firstinspires.ftc.reginald.Localizer;
import org.firstinspires.ftc.reginald.Robot;
import org.firstinspires.ftc.reginald.Turntable;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.SubSystem;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private final Alliance alliance;
    private final Class<? extends SubSystem>[] needs;
    protected Robot robot;

    private Hardware injectedHardware = null;

    private Telemetry driverStationTelemetry = null;

    @SafeVarargs
    protected OpMode(Alliance alliance, Class<? extends SubSystem>... needs) {
        this.alliance = alliance;
        this.needs = needs;
    }

    public Alliance alliance() {
        return alliance;
    }

    public void useHardware(Hardware hardware) {
        this.injectedHardware = hardware;
    }

    protected Hardware hardware() {
        return injectedHardware != null ? injectedHardware : Hardware.fromHardwareMap(hardwareMap);
    }

    @Override
    public void init() {
        Hardware hardware = hardware();

        if (driverStationTelemetry == null) {
            driverStationTelemetry = telemetry;
        }
        telemetry = new MultipleTelemetry(driverStationTelemetry, hardware.dashboard().telemetry());
        robot = new Robot(hardware, alliance, telemetry, new Gamepads(gamepad1, gamepad2), needs);
    }

    public String where() {
        return getClass().getName();
    }

    @Override
    public final void loop() {
        handleTelemetryToggles();
        robot.loop();
        onLoop();
    }

    protected void onLoop() {}

    private void handleTelemetryToggles() {
        if (gamepad2.crossWasPressed()) {
            robot.channels.toggle(Drive.CHANNEL, Localizer.CHANNEL);
        }
        if (gamepad2.squareWasPressed()) {
            robot.channels.toggle(Turntable.CHANNEL, Launcher.CHANNEL);
        }
        if (gamepad2.circleWasPressed()) {
            robot.channels.toggle(Camera.CHANNEL, Brain.CHANNEL);
        }
    }
}
