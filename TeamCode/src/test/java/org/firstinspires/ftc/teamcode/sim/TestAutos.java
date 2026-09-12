package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.RelativeAutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

/**
 * Tiny autos for exercising the simulator itself. They live outside the auto package, so the
 * catalog never lists them on the real bench.
 */
public final class TestAutos {
    private TestAutos() {
    }

    /**
     * An auto whose single step finishes on its third loop.
     */
    @Autonomous(name = "Count to three", group = "Test")
    public static class ThreeLoopAuto extends RelativeAutoOp {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("count to three", () -> {
            }, () -> ++loops >= 3);
        }
    }

    /**
     * An auto that prints to System.out from its step, the way student code does.
     */
    @Autonomous(name = "Chatty", group = "Test")
    public static class ChattyAuto extends RelativeAutoOp {
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
    public static class HangingAuto extends RelativeAutoOp {
        @Override
        public PlanPart getPlan() {
            return new Step("hang", () -> {
                while (true) {
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        // keep hanging: the point is that nothing in-process can stop this
                    }
                }
            }, () -> false);
        }
    }

    @Autonomous(name = "Never done", group = "Test")
    public static class NeverDoneAuto extends RelativeAutoOp {
        @Override
        public PlanPart getPlan() {
            return new Step("forever", () -> {
            }, () -> false);
        }
    }
}
