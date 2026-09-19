package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.opmode.OpMode;

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

        public StickTeleOp() {
            super(Alliance.RED);
        }

        @Override
        protected void onLoop() {
            if (gamepad1.crossWasPressed()) {
                presses++;
            }
            robot.drive.manual(-gamepad1.left_stick_y, 0, 0);
        }
    }

    @TeleOp(name = "Drive intents", group = "Test")
    public static class DriveIntentsTeleOp extends OpMode {
        public static final double PHASE_SECONDS = 0.4;

        public static final double SECONDS = PHASE_SECONDS * 5;

        private long startedAtNanos;

        public DriveIntentsTeleOp() {
            super(Alliance.RELATIVE);
        }

        @Override
        public void init() {
            super.init();
            startedAtNanos = robot.clock.getAsLong();
        }

        @Override
        protected void onLoop() {
            double elapsed = (robot.clock.getAsLong() - startedAtNanos) / 1e9;
            int phase = (int) (elapsed / PHASE_SECONDS);
            switch (phase) {
                case 0:
                    robot.drive.manual(0.6f, 0, 0);
                    break;
                case 1:
                    robot.drive.manual(0, 0.6f, 0);
                    break;
                case 2:
                    robot.drive.manual(0, 0, 0.6f);
                    break;
                case 3:
                    robot.drive.manual(0.5f, 0.5f, 0.5f);
                    break;
                default:
                    robot.drive.toward(robot.nav.pose(24, 12, Math.PI / 4));
                    break;
            }
        }
    }
}
