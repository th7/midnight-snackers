package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Test;

public class EachRobotKeepsToItsOwnPackageTest {
    private static final String SHARED_PACKAGE = "org.firstinspires.ftc.teamcode";

    private static List<Class<?>> mainClassesUnder(String packageName) {
        return Classpath.classesUnder(packageName).stream()
                .filter(type -> !Classpath.isTestClass(type))
                .collect(Collectors.toList());
    }

    private static boolean isAnOpMode(Class<?> type) {
        if (type.getAnnotation(Autonomous.class) != null || type.getAnnotation(TeleOp.class) != null) {
            return true;
        }
        for (Method method : type.getDeclaredMethods()) {
            if (method.getAnnotation(OpModeRegistrar.class) != null) {
                return true;
            }
        }
        return false;
    }

    @Test
    public void eachRobotHasCodeInItsOwnPackage() {
        for (TeamRobot robot : TeamRobot.values()) {
            assertFalse(
                    robot.displayName() + " has no code in " + robot.javaPackage(),
                    mainClassesUnder(robot.javaPackage()).isEmpty());
        }
    }

    @Test
    public void everyOpModeIsInARobotsPackageSoThatRobotsDriversAndSimulatorFindIt() {
        List<String> shared = new ArrayList<>();
        for (Class<?> type : mainClassesUnder(SHARED_PACKAGE)) {
            if (isAnOpMode(type)) {
                shared.add(type.getName());
            }
        }

        assertEquals("op modes outside every robot's package", List.of(), shared);
    }

    @Test
    public void nothingOutsideARobotsPackageUsesItsCode() {
        MainSources sources = MainSources.compiled();
        List<String> reachingIn = new ArrayList<>();
        for (TeamRobot robot : TeamRobot.values()) {
            String own = robot.javaPackage().replace('.', '/') + "/";
            for (Class<?> type : mainClassesUnder(robot.javaPackage())) {
                for (MainSources.Reference reference : sources.referencesTo(type.getName())) {
                    if (!reference.file().startsWith(own)) {
                        reachingIn.add(reference + " uses " + robot.displayName() + "'s " + type.getSimpleName());
                    }
                }
            }
        }

        assertEquals(List.of(), reachingIn);
    }
}
