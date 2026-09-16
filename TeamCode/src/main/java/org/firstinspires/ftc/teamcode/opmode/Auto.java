package org.firstinspires.ftc.teamcode.opmode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.base.Alliance;

/**
 * Makes a plan method of {@link Plans} an autonomous op mode on the driver station. The method
 * must be public, take nothing, and return a plan part. Annotate it once per alliance it plays for;
 * {@link PlanOpModes} registers each as its own op mode.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Repeatable(Auto.Autos.class)
public @interface Auto {
    /** The name on the driver station; the method's own name when left empty. */
    String name() default "";

    String group() default "Autonomous";

    Alliance alliance();

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.METHOD)
    @interface Autos {
        Auto[] value();
    }
}
