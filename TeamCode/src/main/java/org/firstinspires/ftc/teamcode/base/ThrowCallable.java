package org.firstinspires.ftc.teamcode.base;

import java.lang.reflect.InvocationTargetException;

public interface ThrowCallable {
    void call() throws IllegalAccessException, InstantiationException, NoSuchMethodException, InvocationTargetException;
}
