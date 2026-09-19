package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class KeyTest {
    private static final Path ROOT = Paths.get("/srv/checkout").toAbsolutePath();

    @Test
    public void theOnlyWaysToMakeAKeyAreTheTwoCheckedOnes() {
        for (Constructor<?> constructor : Key.class.getDeclaredConstructors()) {
            assertTrue(
                    "Key's constructors must be private, or a request string could become one directly",
                    Modifier.isPrivate(constructor.getModifiers()));
        }

        List<String> factories = new ArrayList<>();
        for (Method method : Key.class.getDeclaredMethods()) {
            if (Modifier.isStatic(method.getModifiers()) && !method.isSynthetic()) {
                factories.add(method.getName());
            }
        }

        assertEquals("of makes one from a real path, under makes one from a checked string", 2, factories.size());
        assertTrue(factories.contains("of"));
        assertTrue(factories.contains("under"));
    }

    @Test
    public void aRealPathUnderTheRootBecomesTheRootRelativePathUsersName() {
        Key key = Key.of(ROOT, ROOT.resolve("TeamCode/src/main/java/Plans.java"));

        assertEquals("TeamCode/src/main/java/Plans.java", key.path());
        assertTrue(key.isJava());
    }

    @Test
    public void aKeyResolvesAgainstWhicheverWorktreeIsAsked() {
        Key key = Key.of(ROOT, ROOT.resolve("TeamCode/Plans.java"));
        Path ada = Paths.get("/state/worktrees/ada").toAbsolutePath();

        assertEquals(ada.resolve("TeamCode/Plans.java"), key.under(ada));
    }

    @Test
    public void aPathThatClimbsOutOfTheRootIsNoKey() {
        assertTrue(Key.under(ROOT, "../../etc/passwd").isEmpty());
        assertTrue(Key.under(ROOT, "TeamCode/../../etc/passwd").isEmpty());
        assertTrue(Key.under(ROOT, "..").isEmpty());
    }

    @Test
    public void anAbsolutePathIsNoKey() {
        assertTrue(Key.under(ROOT, "/etc/passwd").isEmpty());
        assertTrue(Key.under(ROOT, "\\windows\\system32").isEmpty());
        assertTrue(Key.under(ROOT, "C:/windows").isEmpty());
    }

    @Test
    public void theRootItselfAndNothingAtAllAreNoKey() {
        assertTrue(Key.under(ROOT, "").isEmpty());
        assertTrue(Key.under(ROOT, null).isEmpty());
        assertTrue(Key.under(ROOT, ".").isEmpty());
    }

    @Test
    public void aTidyRelativePathIsAKeyAndKeepsItsSpelling() {
        assertEquals(
                "TeamCode/Plans.java",
                Key.under(ROOT, "TeamCode/Plans.java").get().path());
        assertEquals(
                "a path is normalised on the way in",
                "TeamCode/Plans.java",
                Key.under(ROOT, "TeamCode/./src/../Plans.java").get().path());
    }

    @Test
    public void keysAreComparedByTheirPath() {
        Key one = Key.of(ROOT, ROOT.resolve("a/b.java"));
        Key same = Key.under(ROOT, "a/b.java").get();
        Key other = Key.under(ROOT, "a/c.java").get();

        assertEquals(one, same);
        assertEquals(one.hashCode(), same.hashCode());
        assertFalse(one.equals(other));
        assertTrue(one.compareTo(other) < 0);
    }
}
