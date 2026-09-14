package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.base.Auto;
import org.firstinspires.ftc.teamcode.base.PlanOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

/**
 * Registers an autonomous op mode for every {@link Auto} annotation in {@link Plans}. The robot
 * controller calls {@link #register(OpModeManager)} at startup, as it does every registrar.
 */
public final class PlanOpModes {
    private PlanOpModes() {}

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        register(manager, Plans.class);
    }

    /**
     * Registers the {@link Auto} plans declared by {@code plans}. Each op mode calls its method on
     * the {@link Plans} it builds, so only {@link Plans} itself makes sense on the robot.
     *
     * @throws IllegalStateException for an annotation the robot controller could not honour: a
     *                               method that cannot be called for a plan, or a name used twice
     */
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
