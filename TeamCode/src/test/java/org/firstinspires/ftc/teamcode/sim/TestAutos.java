package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.reginald.framework.Discovered;
import org.firstinspires.ftc.reginald.framework.OpMode;
import org.firstinspires.ftc.teamcode.Classpath;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.RunsAPlan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public final class TestAutos {
    private TestAutos() {}

    /**
     * What a test auto is: an op mode of Reginald's that runs the one plan it makes. The robot's own
     * autos are {@code @Auto} methods, which the robot registers itself; a test names its autos as
     * classes, so the simulator can be handed each one on its own. Each drives Reginald as the robot
     * builds it: every subsystem of Reginald's code is built and ticks.
     */
    public abstract static class TestAuto extends OpMode implements RunsAPlan {
        private static final Discovered REGINALD =
                Discovered.of(Classpath.classesUnder("org.firstinspires.ftc.reginald"));

        private PlanRunner planRunner;

        protected TestAuto() {
            super(Alliance.RELATIVE);
            found(REGINALD);
        }

        public abstract PlanPart getPlan();

        @Override
        public void init() {
            super.init();
            planRunner = new PlanRunner();
            planRunner.run(getPlan());
        }

        @Override
        protected void onLoop() {
            planRunner.loop();
            telemetry.addData("Current Step:", planRunner.currentStep());
        }

        @Override
        public boolean done() {
            return planRunner.done();
        }

        @Override
        public String currentStep() {
            return planRunner.currentStep();
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
            return Step.waitFor("wait", SECONDS, hardware().nanoClock());
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
