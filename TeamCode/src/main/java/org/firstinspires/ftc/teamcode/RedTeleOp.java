package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleOp", group = "TeleOp")
public class RedTeleOp extends TeleOp {


    @Override
    Vector2d launchTarget() {
        return drive.redLaunchTarget;
    }
}
