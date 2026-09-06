package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleOp", group = "TeleOp")
public class RedTeleOp extends TeleOp {
    @Override
    protected Nav getNav(MecanumDrive mecanumDrive) {
        return Nav.red(mecanumDrive, runtime, telemetry);
    }
}
