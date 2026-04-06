package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleOp", group = "TeleOp")
public class RedTeleOp extends TeleOp {
    @Override
    protected Nav getNav() {
        return Nav.red(hardwareMap, runtime, telemetry);
    }
}
