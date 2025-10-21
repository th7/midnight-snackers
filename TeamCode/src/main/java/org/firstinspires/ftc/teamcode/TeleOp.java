package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="TeleOp")
public class TeleOp extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void init() {
        super.init();
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("TeleOp.init()", true);
    }

    @Override
    public void loop() {
        super.loop();

        if(gamepad1.dpadUpWasPressed()) { launcher.increasePower(); }
        if(gamepad1.dpadDownWasPressed()) { launcher.decreasePower(); }
        if(gamepad1.squareWasPressed()) { launcher.setCloseLaunchPower(); }
        if(gamepad1.circleWasPressed()) { launcher.setFarLaunchPower(); }
        if(gamepad1.xWasPressed()) { launcher.noPower(); }
        if(gamepad1.triangleWasPressed()) { launcher.launch(); }
        if(gamepad1.rightBumperWasPressed()) { launcher.increaseGatePosition(); }
        if(gamepad1.leftBumperWasPressed()){ launcher.decreaseGatePosition(); }

//        if (gamepad1.triangle) { Arm.armToFloor(); } else if (gamepad1.x) { Arm.armToAboveFloor(); }
//        if (gamepad1.circle) { Arm.armToLowBasket(); } else if (gamepad1.square) { Arm.armToFloorForwards(); }
////        if (gamepad1.dpad_left) { Arm.wristOut(); } else if (gamepad1.dpad_right) { Arm.wristIn(); } else { Arm.wristStop(); }
//        if (gamepad1.right_trigger > 0) { Arm.openClaw(); } else if (gamepad1.left_trigger > 0) { Arm.closeClaw(); }
//        if (gamepad1.dpad_down) { Arm.manualDown(); } else if (gamepad1.dpad_up) { Arm.manualUp(); }

        setTelemetry();
        MoveData straight = MoveData.straight(-gamepad1.left_stick_y, 0f, 1f);
        MoveData strafe = MoveData.strafe(-gamepad1.left_stick_x,0f, 1f);
        MoveData turn = MoveData.turn(-gamepad1.right_stick_x, 0f, 1f);
        MoveData moveData = straight.add(strafe, turn);
        leftFront.setPower(moveData.frontLeftPower);
        rightFront.setPower(moveData.frontRightPower);
        leftBack.setPower(moveData.rearLeftPower);
        rightBack.setPower(moveData.rearRightPower);
        telemetry.addData("frontLeftPower", moveData.frontLeftPower);
        telemetry.addData("frontRightPower", moveData.frontRightPower);
        telemetry.addData("rearLeftPower", moveData.rearLeftPower);
        telemetry.addData("rearRightPower", moveData.rearRightPower);
    }

    private void setTelemetry() {
        telemetry.addData("gameStickLeftX", gamepad1.left_stick_x);
        telemetry.addData("gameStickLeftY", gamepad1.left_stick_y);
        telemetry.addData("gameStickRightX", gamepad1.right_stick_x);
        telemetry.addData("gameStickRightY", gamepad1.right_stick_y);
        telemetry.addData("circle", gamepad1.circle);
        telemetry.addData("leftTrigger", gamepad1.left_trigger);
        telemetry.addData("rightTrigger", gamepad1.right_trigger);
        telemetry.addData("leftBumper", gamepad1.left_bumper);
        telemetry.addData("rightBumper", gamepad1.right_bumper);
        telemetry.addData("dpad_up", gamepad1.dpad_up);
        telemetry.addData("dpad_down", gamepad1.dpad_down);
        telemetry.update();
    }


}
