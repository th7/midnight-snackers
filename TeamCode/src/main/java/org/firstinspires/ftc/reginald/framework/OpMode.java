package org.firstinspires.ftc.reginald.framework;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Gamepads;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private final Alliance alliance;
    private Discovered discovered = Discovered.NOTHING;
    protected Robot robot;

    private Hardware injectedHardware = null;

    private Telemetry driverStationTelemetry = null;

    protected OpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance alliance() {
        return alliance;
    }

    public void found(Discovered discovered) {
        this.discovered = discovered;
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
        robot = new Robot(hardware, alliance, telemetry, new Gamepads(gamepad1, gamepad2), discovered);
    }

    public String where() {
        return getClass().getName();
    }

    @Override
    public final void loop() {
        showTheNextChannelWhenAsked();
        beforeTheRobotTicks();
        robot.loop();
        onLoop();
    }

    void beforeTheRobotTicks() {}

    protected void onLoop() {}

    private void showTheNextChannelWhenAsked() {
        if (gamepad2.shareWasPressed()) {
            robot.channels.step();
        }
        telemetry.addData("Telemetry", robot.channels.showing().orElse("off") + ", share on gamepad 2 for the next");
    }
}
