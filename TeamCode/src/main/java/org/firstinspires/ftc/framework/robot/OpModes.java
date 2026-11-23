package org.firstinspires.ftc.framework.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.framework.plans.PlanPart;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public final class OpModes {
    public static final String TELEOP_GROUP = "TeleOp";

    private OpModes() {}

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        register(manager, Discovered.inThisApp());
    }

    public static void register(OpModeManager manager, Discovered discovered) {
        Map<String, String> named = new HashMap<>();
        teleOp(manager, named, "BlueTeleOp", Alliance.BLUE, discovered);
        teleOp(manager, named, "RedTeleOp", Alliance.RED, discovered);
        for (Method method : discovered.autos()) {
            String where = PlanOp.whereIs(method);
            if (!Modifier.isPublic(method.getModifiers()) || !PlanPart.class.isAssignableFrom(method.getReturnType())) {
                throw new IllegalStateException(
                        "@Auto needs a public method that returns a PlanPart; " + where + " is not one");
            }
            for (Auto auto : method.getAnnotationsByType(Auto.class)) {
                String name = auto.name().isEmpty() ? method.getName() : auto.name();
                claim(named, name, where);
                PlanOp planOp = new PlanOp(auto.alliance(), method);
                planOp.found(discovered);
                manager.register(meta(OpModeMeta.Flavor.AUTONOMOUS, name, auto.group()), planOp);
            }
        }
    }

    private static void teleOp(
            OpModeManager manager, Map<String, String> named, String name, Alliance alliance, Discovered discovered) {
        claim(named, name, TeleOp.WHERE);
        TeleOp teleOp = new TeleOp(alliance);
        teleOp.found(discovered);
        manager.register(meta(OpModeMeta.Flavor.TELEOP, name, TELEOP_GROUP), teleOp);
    }

    private static void claim(Map<String, String> named, String name, String where) {
        String other = named.put(name, where);
        if (other != null) {
            throw new IllegalStateException("two op modes are named " + name + ": " + other + " and " + where);
        }
    }

    private static OpModeMeta meta(OpModeMeta.Flavor flavor, String name, String group) {
        return new OpModeMeta.Builder()
                .setFlavor(flavor)
                .setName(name)
                .setGroup(group)
                .build();
    }
}
