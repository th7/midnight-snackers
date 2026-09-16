package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.opmode.OpMode;

/**
 * Tiny TeleOps for exercising the simulator itself. Nested classes, so the catalog never lists
 * them on the real bench.
 */
public final class TestTeleOps {
    private TestTeleOps() {}

    /**
     * Drives straight from gamepad 1's left stick and counts the presses of its cross button.
     */
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

    /**
     * Gives the drive every intent a driver can, on a schedule of its own rather than from a
     * gamepad, so a run of it is the same tick for tick: each axis alone, then all three at once
     * hard enough that a wheel saturates, and then a pose to steer to. No driver station is
     * involved, so nothing here depends on when a thread got to set a stick.
     *
     * <p>What it is for is {@link SimTrace}: the powers it puts on the wheels are the drive's
     * whole command path, and a trace of them says whether that path still answers an intent the
     * way it did.
     */
    @TeleOp(name = "Drive intents", group = "Test")
    public static class DriveIntentsTeleOp extends OpMode {
        /** How long each intent below is held, in seconds of the robot's clock. */
        public static final double PHASE_SECONDS = 0.4;
        /** Every intent, held {@link #PHASE_SECONDS} each; the run is this long in total. */
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
                    // All three at once, past what one wheel can take: the case where how the drive
                    // saturates a mixed command is the whole of the answer.
                    robot.drive.manual(0.5f, 0.5f, 0.5f);
                    break;
                default:
                    // The drive steers itself, on the pose the localizer believes: FastDrive's
                    // three controllers, and the powers they ask for.
                    robot.drive.toward(robot.nav.pose(24, 12, Math.PI / 4));
                    break;
            }
        }
    }
}
