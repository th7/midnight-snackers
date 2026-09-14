package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

/**
 * Tiny autos for exercising the simulator itself. Nested classes, so the catalog never lists
 * them on the real bench.
 */
public final class TestAutos {
    private TestAutos() {}

    /** An auto for no alliance in particular, like every auto here. */
    public abstract static class TestAuto extends AutoOp {
        protected TestAuto() {
            super(Alliance.RELATIVE);
        }
    }

    /**
     * An auto whose single step finishes on its third loop.
     */
    @Autonomous(name = "Count to three", group = "Test")
    public static class ThreeLoopAuto extends TestAuto {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("count to three", () -> {}, () -> ++loops >= 3);
        }
    }

    /**
     * An auto that prints to System.out from its step, the way student code does.
     */
    @Autonomous(name = "Chatty", group = "Test")
    public static class ChattyAuto extends TestAuto {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("chat", () -> System.out.println("hello from the op mode"), () -> ++loops >= 2);
        }
    }

    /**
     * An auto whose loop never returns, so no cooperative timeout can end it.
     */
    @Autonomous(name = "Hangs", group = "Test")
    public static class HangingAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step(
                    "hang",
                    () -> {
                        while (true) {
                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException e) {
                                // keep hanging: the point is that nothing in-process can stop this
                            }
                        }
                    },
                    () -> false);
        }
    }

    /** An auto that waits {@link #SECONDS} on the robot's clock, then is done. */
    @Autonomous(name = "Wait two seconds", group = "Test")
    public static class WaitingAuto extends TestAuto {
        public static final double SECONDS = 2;

        @Override
        public PlanPart getPlan() {
            return Step.waitFor("wait", SECONDS, robot.clock);
        }
    }

    @Autonomous(name = "Never done", group = "Test")
    public static class NeverDoneAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step("forever", () -> {}, () -> false);
        }
    }

    /**
     * An auto whose single step waits until the test calls {@link #release()}, so the test decides
     * when the run ends instead of racing a timeout.
     */
    @Autonomous(name = "Gated", group = "Test")
    public static class GatedAuto extends TestAuto {
        public static final String STEP = "wait for the gate";
        private final AtomicBoolean released = new AtomicBoolean(false);

        public void release() {
            released.set(true);
        }

        @Override
        public PlanPart getPlan() {
            return new Step(STEP, () -> {}, released::get);
        }
    }
}
