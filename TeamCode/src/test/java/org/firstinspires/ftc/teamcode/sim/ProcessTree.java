package org.firstinspires.ftc.teamcode.sim;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * The processes a process started, and theirs. {@code java.lang.ProcessHandle} is what knows, and
 * the tests compile against {@code android.jar}, which has none, so it is reached here by name and
 * nowhere else. The JVM that runs them always has it: a missing method is a broken JDK, and throws.
 */
public final class ProcessTree {
    private static final Class<?> HANDLE = handleClass();
    private static final Method CURRENT = method("current");
    private static final Method OF = method("of", long.class);
    private static final Method PID = method("pid");
    private static final Method CHILDREN = method("children");
    private static final Method DESCENDANTS = method("descendants");
    private static final Method DESTROY_FORCIBLY = method("destroyForcibly");
    private static final Method ON_EXIT = method("onExit");
    private static final Method TO_HANDLE = processMethod("toHandle");

    private ProcessTree() {}

    /** The pid of a process this JVM started. */
    public static long pid(Process process) {
        return (long) call(PID, call(TO_HANDLE, process));
    }

    /** Kills everything a process started, and everything those started, but not the process. */
    public static void killDescendantsOf(Process process) {
        killAll(descendants(call(TO_HANDLE, process)));
    }

    /** Kills everything this JVM started, and everything those started. */
    public static void killDescendantsOfThisJvm() {
        killAll(descendants(call(CURRENT, null)));
    }

    /** The pids of the processes this JVM started itself. */
    public static List<Long> childrenOfThisJvm() {
        return streamOf(call(CHILDREN, call(CURRENT, null)))
                .map(handle -> (long) call(PID, handle))
                .collect(Collectors.toList());
    }

    /** Whether the process with this pid has ended, or ends within this long. */
    public static boolean endsWithin(long pid, double seconds) throws InterruptedException {
        Optional<?> handle = (Optional<?>) call(OF, null, pid);
        if (handle.isEmpty()) {
            return true;
        }
        try {
            ((CompletableFuture<?>) call(ON_EXIT, handle.get())).get((long) (seconds * 1000), TimeUnit.MILLISECONDS);
            return true;
        } catch (TimeoutException e) {
            return false;
        } catch (ExecutionException e) {
            throw new IllegalStateException(e);
        }
    }

    /** Kills the process with this pid, if it is still running. */
    public static void kill(long pid) {
        ((Optional<?>) call(OF, null, pid)).ifPresent(handle -> call(DESTROY_FORCIBLY, handle));
    }

    private static List<Object> descendants(Object handle) {
        return streamOf(call(DESCENDANTS, handle)).collect(Collectors.toList());
    }

    private static void killAll(List<Object> handles) {
        for (Object handle : handles) {
            call(DESTROY_FORCIBLY, handle);
        }
    }

    @SuppressWarnings("unchecked")
    private static Stream<Object> streamOf(Object stream) {
        return (Stream<Object>) stream;
    }

    private static Object call(Method method, Object on, Object... args) {
        try {
            return method.invoke(on, args);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        } catch (InvocationTargetException e) {
            if (e.getCause() instanceof RuntimeException) {
                throw (RuntimeException) e.getCause();
            }
            throw new IllegalStateException(e.getCause());
        }
    }

    private static Class<?> handleClass() {
        try {
            return Class.forName("java.lang.ProcessHandle");
        } catch (ClassNotFoundException e) {
            throw new IllegalStateException("this JVM has no ProcessHandle", e);
        }
    }

    private static Method method(String name, Class<?>... parameters) {
        try {
            return HANDLE.getMethod(name, parameters);
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException("this JVM's ProcessHandle has no " + name, e);
        }
    }

    private static Method processMethod(String name) {
        try {
            return Process.class.getMethod(name);
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException("this JVM's Process has no " + name, e);
        }
    }
}
