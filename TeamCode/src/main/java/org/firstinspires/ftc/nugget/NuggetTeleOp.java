package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Nugget TeleOp", group = "Nugget")
public class NuggetTeleOp extends NuggetOpMode {
    private TankDrive drive;

    @Override
    public void init() {
        drive = new TankDrive(hardware());
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }
}
