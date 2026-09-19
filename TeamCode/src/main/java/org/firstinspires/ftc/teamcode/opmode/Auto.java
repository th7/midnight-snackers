package org.firstinspires.ftc.teamcode.opmode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.firstinspires.ftc.teamcode.base.Alliance;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@Repeatable(Auto.Autos.class)
public @interface Auto {
    String name() default "";

    String group() default "Autonomous";

    Alliance alliance();

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.METHOD)
    @interface Autos {
        Auto[] value();
    }
}
