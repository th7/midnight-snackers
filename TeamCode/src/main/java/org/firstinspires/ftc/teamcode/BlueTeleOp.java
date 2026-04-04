package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends TeleOp {
    @Override
    Vector2d launchTarget() {
        return drive.blueLaunchTarget;
    }
}
