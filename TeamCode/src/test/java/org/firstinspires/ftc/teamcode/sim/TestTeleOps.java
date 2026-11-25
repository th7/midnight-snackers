package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.framework.robot.Alliance;
import org.firstinspires.ftc.framework.robot.OpMode;

public final class TestTeleOps {
    private TestTeleOps() {}

    @TeleOp(name = "Mid loop", group = "Test")
    public static class MidLoopTeleOp extends OpMode {
        public SimDriverStation station;

        public SimDriverStation.State next = SimDriverStation.State.NEUTRAL;

        public boolean readCross = false;

        public MidLoopTeleOp() {
            super(Alliance.RED);
        }

        @Override
        protected void onLoop() {
            readCross = readCross || gamepad1.cross;
            if (station != null) {
                station.set(1, next);
            }
        }
    }

    @TeleOp(name = "Stick", group = "Test")
    public static class StickTeleOp extends OpMode {
        public int presses = 0;

        private List<DcMotorEx> wheels;

        public StickTeleOp() {
            super(Alliance.RED);
        }

        @Override
        public void init() {
            super.init();
            DcMotorEx leftBack = hardware().motor("leftBack");
            DcMotorEx rightBack = hardware().motor("rightBack");
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            wheels = List.of(
                    hardware().motor("leftFront"),
                    leftBack,
                    rightBack,
                    hardware().motor("rightFront"));
            for (DcMotorEx wheel : wheels) {
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        @Override
        protected void onLoop() {
            if (gamepad1.crossWasPressed()) {
                presses++;
            }
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(-gamepad1.left_stick_y);
            }
        }
    }

    @TeleOp(name = "Slow machine", group = "Test")
    public static class SlowMachineTeleOp extends OpMode {
        public static final double REAL_SECONDS_PER_LOOP = 0.25;

        public SlowMachineTeleOp() {
            super(Alliance.RELATIVE);
        }

        @Override
        protected void onLoop() {
            try {
                Thread.sleep((long) (REAL_SECONDS_PER_LOOP * 1000));
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
