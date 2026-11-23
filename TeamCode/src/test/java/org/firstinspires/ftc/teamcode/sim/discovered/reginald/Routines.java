package org.firstinspires.ftc.teamcode.sim.discovered.reginald;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos;

public final class Routines {
    private Routines() {}

    public static final class Routine extends TestAutos.TestAuto {
        private final String where;

        Routine(String where) {
            this.where = where;
        }

        @Override
        public PlanPart getPlan() {
            return new Step("nothing", () -> {}, () -> true);
        }

        @Override
        public String where() {
            return where;
        }
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (String name : new String[] {"forward", "back"}) {
            manager.register(
                    new OpModeMeta.Builder()
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .setName(name)
                            .setGroup("Routines")
                            .build(),
                    new Routine("Routines." + name + "()"));
        }
    }
}
