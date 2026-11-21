package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.Test;

/**
 * The client's suite runs on Linux and on Windows, and a test that cannot mean anything on one of
 * them passes itself over there only if passed-over.json names it, with why, under that platform:
 * the cost judge in TeamCode/suite-cost.gradle prints the list and fails on a skip it does not name.
 * The list is only read on the platform it names, so a test renamed or deleted on Linux would
 * leave a name nothing checks; this holds every name to a test that is there.
 */
public class ThePassedOverListNamesTestsThatExistTest {
    private static final String LIST = "passed-over.json";

    @Test
    public void everyTestTheListPassesOverIsATestThatExistsAndSaysWhy() throws IOException {
        JsonObject list = listed();

        assertEquals(List.of(), unexplained(list));
        assertEquals(List.of(), noLongerThere(list));
    }

    @Test
    public void theRuleWouldNoticeANameThatIsNoLongerThere() {
        JsonObject list = new Gson()
                .fromJson(
                        "{\"windows\": {\"" + getClass().getName() + ".aTestNobodyWrote\": \"it was renamed\"," + " \""
                                + getClass().getName() + ".theRuleWouldNoticeANameThatIsNoLongerThere\": \"here\"}}",
                        JsonObject.class);

        assertEquals(List.of(getClass().getName() + ".aTestNobodyWrote"), noLongerThere(list));
    }

    @Test
    public void theRuleWouldNoticeANameThatSaysNotWhy() {
        JsonObject list =
                new Gson().fromJson("{\"windows\": {\"" + getClass().getName() + ".aTest\": \"\"}}", JsonObject.class);

        assertFalse(unexplained(list).isEmpty());
    }

    private static JsonObject listed() throws IOException {
        try (InputStream in = ThePassedOverListNamesTestsThatExistTest.class.getResourceAsStream(LIST)) {
            assertNotNull("there is no " + LIST + " beside the client's suite budget", in);
            return new Gson().fromJson(new String(in.readAllBytes(), StandardCharsets.UTF_8), JsonObject.class);
        }
    }

    private static List<String> unexplained(JsonObject list) {
        List<String> unexplained = new ArrayList<>();
        for (Map.Entry<String, JsonElement> platform : list.entrySet()) {
            for (Map.Entry<String, JsonElement> test :
                    platform.getValue().getAsJsonObject().entrySet()) {
                if (test.getValue().getAsString().isBlank()) {
                    unexplained.add(platform.getKey() + ": " + test.getKey());
                }
            }
        }
        return unexplained;
    }

    private static List<String> noLongerThere(JsonObject list) {
        List<String> gone = new ArrayList<>();
        for (Map.Entry<String, JsonElement> platform : list.entrySet()) {
            for (Map.Entry<String, JsonElement> test :
                    platform.getValue().getAsJsonObject().entrySet()) {
                if (!isATest(test.getKey())) {
                    gone.add(test.getKey());
                }
            }
        }
        return gone;
    }

    private static boolean isATest(String name) {
        int dot = name.lastIndexOf('.');
        if (dot < 0) {
            return false;
        }
        try {
            Method method = Class.forName(name.substring(0, dot)).getMethod(name.substring(dot + 1));
            return method.isAnnotationPresent(Test.class);
        } catch (ClassNotFoundException | NoSuchMethodException e) {
            return false;
        }
    }
}
