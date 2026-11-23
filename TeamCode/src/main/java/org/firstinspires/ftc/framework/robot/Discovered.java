package org.firstinspires.ftc.framework.robot;

import dalvik.system.DexFile;
import java.io.IOException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.TreeMap;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public final class Discovered {
    public static final List<String> THE_ROBOTS_CODE =
            List.of("org.firstinspires.ftc.reginald.", "org.firstinspires.ftc.framework.");

    public static final Discovered NOTHING = new Discovered(List.of());

    private final List<Class<?>> classes;

    private Discovered(List<Class<?>> classes) {
        this.classes = classes;
    }

    public static Discovered of(Class<?>... classes) {
        return of(Arrays.asList(classes));
    }

    public static Discovered of(Collection<? extends Class<?>> classes) {
        TreeMap<String, Class<?>> byName = new TreeMap<>();
        for (Class<?> found : classes) {
            byName.put(found.getName(), found);
        }
        return new Discovered(List.copyOf(byName.values()));
    }

    public static Discovered inThisApp() {
        ClassLoader loader = Discovered.class.getClassLoader();
        List<Class<?>> found = new ArrayList<>();
        try {
            DexFile app = new DexFile(AppUtil.getInstance().getApplication().getPackageCodePath());
            for (String name : Collections.list(app.entries())) {
                if (isTheRobots(name)) {
                    found.add(Class.forName(name, false, loader));
                }
            }
        } catch (IOException | ClassNotFoundException failed) {
            throw new IllegalStateException("the robot could not list the classes of its own code", failed);
        }
        return of(found);
    }

    private static boolean isTheRobots(String name) {
        for (String prefix : THE_ROBOTS_CODE) {
            if (name.startsWith(prefix)) {
                return true;
            }
        }
        return false;
    }

    public boolean has(Class<?> kind) {
        return classes.contains(kind);
    }

    public List<Class<? extends SubSystem>> subsystems() {
        List<Class<? extends SubSystem>> subsystems = new ArrayList<>();
        for (Class<?> found : concrete()) {
            if (SubSystem.class.isAssignableFrom(found)) {
                subsystems.add(found.asSubclass(SubSystem.class));
            }
        }
        return subsystems;
    }

    public List<Method> autos() {
        List<Method> autos = new ArrayList<>();
        for (Class<?> found : classes) {
            List<Method> declared = new ArrayList<>(Arrays.asList(found.getDeclaredMethods()));
            declared.sort(Comparator.comparing(Method::getName));
            for (Method method : declared) {
                if (method.getAnnotationsByType(Auto.class).length > 0) {
                    autos.add(method);
                }
            }
        }
        return autos;
    }

    public List<Class<?>> implementing(Class<?> kind) {
        List<Class<?>> implementing = new ArrayList<>();
        for (Class<?> found : concrete()) {
            if (kind.isAssignableFrom(found)) {
                implementing.add(found);
            }
        }
        return implementing;
    }

    private List<Class<?>> concrete() {
        List<Class<?>> concrete = new ArrayList<>();
        for (Class<?> found : classes) {
            if (!found.isInterface() && !Modifier.isAbstract(found.getModifiers())) {
                concrete.add(found);
            }
        }
        return concrete;
    }
}
