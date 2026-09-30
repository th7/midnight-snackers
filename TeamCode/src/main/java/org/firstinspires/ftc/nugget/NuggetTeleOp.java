package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Nugget TeleOp", group = "Nugget")
public class NuggetTeleOp extends OpMode {
    private NuggetHardware injectedHardware = null;

    private TankDrive drive;

    public void useHardware(NuggetHardware hardware) {
        this.injectedHardware = hardware;
    }

    @Override
    public void init() {
        drive = new TankDrive(
                injectedHardware != null ? injectedHardware : NuggetHardware.fromHardwareMap(hardwareMap));
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }
}
