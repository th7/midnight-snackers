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
     * Writes the driver station from inside its own loop, the way the controller page writes while
     * a loop is running. What the gamepad read this loop and what the driver has pressed since are
     * then different, and a tick can only carry one of them.
     */
    @TeleOp(name = "Mid loop", group = "Test")
    public static class MidLoopTeleOp extends OpMode {
        /** The station to write once this loop has read it; none, to leave it alone. */
        public SimDriverStation station;
        /** What to write to gamepad 1 in the middle of the loop. */
        public SimDriverStation.State next = SimDriverStation.State.NEUTRAL;
        /** Whether gamepad 1's cross was ever down on a loop this op mode read. */
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
