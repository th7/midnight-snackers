package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private boolean telemetryOn = false;

    @Override
    public void init() {
        super.init();
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
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

        if (gamepad1.dpadUpWasPressed()) {
            launcher.increasePower();
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcher.decreasePower();
        }
        if (gamepad1.squareWasPressed()) {
            launcher.setCloseLaunchPower();
        }
        if (gamepad1.circleWasPressed()) {
            launcher.setFarLaunchPower();
        }
        if (gamepad1.crossWasPressed()) {
            launcher.noPower();
        }
        if (gamepad1.triangleWasPressed()) {
            launcher.launch();
        }
        if (gamepad1.rightBumperWasPressed()) {
            launcher.increaseGatePosition();
        }
        if (gamepad1.leftBumperWasPressed()) {
            launcher.decreaseGatePosition();
        }

        MoveData straight = MoveData.straight(-gamepad1.left_stick_y, 0f, 1f);
        MoveData strafe = MoveData.strafe(-gamepad1.left_stick_x, 0f, 1f);
        MoveData turn = MoveData.turn(-gamepad1.right_stick_x, 0f, 1f);
        MoveData moveData = straight.add(strafe, turn);
        leftFront.setPower(moveData.frontLeftPower);
        rightFront.setPower(moveData.frontRightPower);
        leftBack.setPower(moveData.rearLeftPower);
        rightBack.setPower(moveData.rearRightPower);

        if (gamepad2.triangleWasPressed()) { telemetryOn = !telemetryOn; }
        if (telemetryOn) {
            telemetry.addData("frontLeftPower", moveData.frontLeftPower);
            telemetry.addData("frontRightPower", moveData.frontRightPower);
            telemetry.addData("rearLeftPower", moveData.rearLeftPower);
            telemetry.addData("rearRightPower", moveData.rearRightPower);
            telemetry.update();
        }
    }
}
