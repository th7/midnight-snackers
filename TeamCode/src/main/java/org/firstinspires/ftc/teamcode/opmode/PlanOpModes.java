package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Set;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

public final class PlanOpModes {
    private PlanOpModes() {}

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        register(manager, Plans.class);
    }

    public static void register(OpModeManager manager, Class<? extends Plans> plans) {
        Method[] methods = plans.getDeclaredMethods();
        Arrays.sort(methods, Comparator.comparing(Method::getName));
        Set<String> names = new HashSet<>();
        for (Method method : methods) {
            Auto[] autos = method.getAnnotationsByType(Auto.class);
            if (autos.length == 0) {
                continue;
            }
            String where = plans.getSimpleName() + "." + method.getName() + "()";
            if (!Modifier.isPublic(method.getModifiers())
                    || method.getParameterCount() != 0
                    || !PlanPart.class.isAssignableFrom(method.getReturnType())) {
                throw new IllegalStateException(
                        "@Auto needs a public method with no parameters that returns a PlanPart; " + where
                                + " is not one");
            }
            for (Auto auto : autos) {
                String name = auto.name().isEmpty() ? method.getName() : auto.name();
                if (!names.add(name)) {
                    throw new IllegalStateException("two @Auto plans are named " + name + "; the second is " + where);
                }
                OpModeMeta meta = new OpModeMeta.Builder()
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setName(name)
                        .setGroup(auto.group())
                        .build();
                manager.register(meta, new PlanOp(auto.alliance(), where, p -> planFrom(method, p)));
            }
        }
    }

    private static PlanPart planFrom(Method method, Plans plans) {
        try {
            return (PlanPart) method.invoke(plans);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("could not call " + method, e);
        } catch (InvocationTargetException e) {
            throw new IllegalStateException(method.getName() + "() failed to make its plan", e.getCause());
        }
    }
}
