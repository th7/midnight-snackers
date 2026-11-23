package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.reginald.hardware.Wheels;
import org.firstinspires.ftc.reginald.opmode.OpMode;
import org.firstinspires.ftc.teamcode.base.Alliance;

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

        private Wheels wheels;

        public StickTeleOp() {
            super(Alliance.RED);
        }

        @Override
        public void init() {
            super.init();
            wheels = new Wheels(hardware());
        }

        @Override
        protected void onLoop() {
            if (gamepad1.crossWasPressed()) {
                presses++;
            }
            wheels.drive(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, 0), 0));
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
