package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.opmode.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public final class TestAutos {
    private TestAutos() {}

    public abstract static class TestAuto extends AutoOp {
        protected TestAuto() {
            super(Alliance.RELATIVE);
        }
    }

    @Autonomous(name = "Count to three", group = "Test")
    public static class ThreeLoopAuto extends TestAuto {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("count to three", () -> {}, () -> ++loops >= 3);
        }
    }

    @Autonomous(name = "Chatty", group = "Test")
    public static class ChattyAuto extends TestAuto {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("chat", () -> System.out.println("hello from the op mode"), () -> ++loops >= 2);
        }
    }

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
                            }
                        }
                    },
                    () -> false);
        }
    }

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
