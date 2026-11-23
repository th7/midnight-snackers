package org.firstinspires.ftc.reginald.framework;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.RunsAPlan;

public final class PlanOp extends OpMode implements RunsAPlan {
    private final Method auto;
    private PlanRunner planRunner;

    PlanOp(Alliance alliance, Method auto) {
        super(alliance);
        this.auto = auto;
    }

    static String whereIs(Method auto) {
        return auto.getDeclaringClass().getSimpleName() + "." + auto.getName() + "()";
    }

    @Override
    public void init() {
        super.init();
        planRunner = new PlanRunner();
        planRunner.run(plan());
    }

    private PlanPart plan() {
        Class<?> declaring = auto.getDeclaringClass();
        Class<?>[] needs = auto.getParameterTypes();
        Object[] parts = new Object[needs.length];
        for (int i = 0; i < needs.length; i++) {
            parts[i] = robot.partFor(needs[i], declaring);
        }
        try {
            Object plans = Modifier.isStatic(auto.getModifiers())
                    ? null
                    : declaring.getConstructor().newInstance();
            return (PlanPart) auto.invoke(plans, parts);
        } catch (InvocationTargetException failed) {
            throw new IllegalStateException(whereIs(auto) + " failed to make its plan", failed.getCause());
        } catch (ReflectiveOperationException failed) {
            throw new IllegalStateException("could not call " + whereIs(auto), failed);
        }
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

    @Override
    public String where() {
        return whereIs(auto);
    }
}
