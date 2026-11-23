package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.RunsAPlan;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.firstinspires.ftc.teamcode.sim.discovered.reginald.RedDriving;
import org.firstinspires.ftc.teamcode.sim.discovered.reginald.Routines;
import org.junit.Test;

public class SimCatalogTest {
    private final SimCatalog catalog = SimCatalog.discover(RedDriving.class.getPackageName());

    @Test
    public void reginaldsCatalogListsNoOpModeThatIsNotReginaldsKind() {
        assertFalse(
                "an op mode in Reginald's package that does not extend Reginald's OpMode is not Reginald's",
                catalog.find("Not Reginald's, among Reginald's").isPresent());
    }

    @Test
    public void reginaldsOpModesAreItsOwnPackage() {
        assertEquals("org.firstinspires.ftc.reginald", SimCatalog.PACKAGE);
    }

    @Test
    public void listsEveryAutoARegistrarInThePackageRegisters() {
        SimCatalog.Entry forward = catalog.find("forward").get();

        assertEquals("auto", forward.kind);
        assertEquals("Routines", forward.group);
        assertEquals("Routines.forward()", forward.where);
        assertEquals("Routines.back()", catalog.find("back").get().where);
        assertTrue(
                "discovered, so another JVM lists the same without being told where to look",
                catalog.sources().isEmpty());
    }

    @Test
    public void listsEveryAnnotatedTeleOpToo() {
        SimCatalog.Entry red = catalog.find("Red driving").get();

        assertEquals("teleop", red.kind);
        assertEquals("Driving", red.group);
        assertEquals("org.firstinspires.ftc.teamcode.sim.discovered.reginald.RedDriving", red.where);
        assertTrue(catalog.find("Blue driving").isPresent());
        assertFalse(
                "an abstract base is no op mode to run",
                catalog.find("Unfinished").isPresent());
        assertFalse(
                "Road Runner's tuning op modes are not ours to simulate",
                catalog.find("Tuning").isPresent());
        assertFalse(
                "nested test op modes never reach the bench",
                SimCatalog.discover(ThreeLoopAuto.class.getPackageName())
                        .find("Count to three")
                        .isPresent());
    }

    @Test
    public void autosComeBeforeTeleOpsEachSortedByName() {
        assertEquals(
                List.of("back", "forward", "Blue driving", "Red driving"),
                catalog.entries().stream().map(e -> e.name).collect(Collectors.toList()));
    }

    @Test
    public void aRegisteredInstanceIsTheOpModeEveryTimeAsOnTheRobotAndAnAnnotatedClassIsBuiltAfresh() {
        SimCatalog.Entry plan = catalog.find("forward").get();
        SimCatalog.Entry teleOp = catalog.find("Red driving").get();

        assertSame(plan.opMode(), plan.opMode());
        assertTrue(plan.opMode() instanceof RunsAPlan);
        assertNotSame(teleOp.opMode(), teleOp.opMode());
        assertTrue(teleOp.opMode() instanceof RedDriving);
        assertFalse(catalog.find("org.example.Nope").isPresent());
    }

    @Test
    public void aFixedCatalogListsTheOpModesAndRegistrarsItIsGivenAndRemembersThem() {
        SimCatalog fixed = SimCatalog.of(StickTeleOp.class, ThreeLoopAuto.class, Routines.class);

        SimCatalog.Entry count = fixed.find("Count to three").get();
        assertEquals("auto", count.kind);
        assertEquals(ThreeLoopAuto.class.getName(), count.where);
        assertEquals("Routines.forward()", fixed.find("forward").get().where);
        SimCatalog.Entry stick = fixed.entries().get(fixed.entries().size() - 1);
        assertEquals("Stick", stick.name);
        assertEquals("teleop", stick.kind);
        assertEquals("Test", stick.group);
        assertEquals(
                List.of(StickTeleOp.class.getName(), ThreeLoopAuto.class.getName(), Routines.class.getName()),
                fixed.sources());
    }

    @Test
    public void aClassThatIsNeitherAnOpModeNorARegistrarCannotBeASource() {
        try {
            SimCatalog.of(String.class);
            fail("String is no source of op modes");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("java.lang.String"));
        }
    }

    @Autonomous(name = "Count to three", group = "Test")
    public static class AnotherCountToThree extends TestAutos.TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step("wait", () -> {}, () -> false);
        }
    }

    @Test
    public void twoOpModesWithOneNameAreRefusedAsTheRobotControllerWould() {
        try {
            SimCatalog.of(ThreeLoopAuto.class, AnotherCountToThree.class);
            fail("the robot controller refuses two op modes with one name");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("Count to three"));
        }
    }

    public static class Broken {
        @OpModeRegistrar
        public static void register(OpModeManager manager) {
            throw new IllegalArgumentException("boom");
        }
    }

    @Test
    public void aRegistrarThatFailsFailsTheCatalogRatherThanListingLess() {
        try {
            SimCatalog.of(Broken.class);
            fail("a registrar that throws would stop the robot controller too");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("Broken"));
            assertEquals("boom", expected.getCause().getMessage());
        }
    }

    @Test
    public void eachOpModeSaysWhichAllianceItPlaysForAndOneThatPlaysForNoneSaysNone() {
        assertEquals("Blue", catalog.find("Blue driving").get().alliance);
        assertEquals("Red", catalog.find("Red driving").get().alliance);
        assertNull(
                "a plan relative to where the robot starts plays for nobody",
                catalog.find("forward").get().alliance);

        SimCatalog parsed = SimCatalog.fromJson(catalog.toJson());

        assertEquals("Red", parsed.find("Red driving").get().alliance);
        assertEquals("Blue", parsed.find("Blue driving").get().alliance);
        assertNull(parsed.find("forward").get().alliance);
    }

    @Test
    public void aListingFromBeforeAlliancesPlaysForNoneAndAnAllianceNobodyPlaysIsRefused() {
        String before = "[{\"name\":\"Old\",\"group\":\"\",\"kind\":\"auto\",\"where\":\"org.example.Old\"}]";
        assertNull(SimCatalog.fromJson(new Gson().fromJson(before, JsonArray.class))
                .find("Old")
                .get()
                .alliance);

        String green = "[{\"name\":\"Odd\",\"group\":\"\",\"kind\":\"auto\",\"where\":\"org.example.Odd\","
                + "\"alliance\":\"Green\"}]";
        try {
            SimCatalog.fromJson(new Gson().fromJson(green, JsonArray.class));
            fail("an alliance nobody plays for is no alliance to stand a driver for");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("Green"));
        }
    }

    @Test
    public void theJsonListingCarriesKindAndWhereAndParsesBackWithNothingToBuild() {
        JsonArray listed = catalog.toJson();
        SimCatalog parsed = SimCatalog.fromJson(listed);

        assertEquals("teleop", parsed.find("Red driving").get().kind);
        assertEquals("Routines.forward()", parsed.find("forward").get().where);
        assertTrue(parsed.sources().isEmpty());

        JsonArray json = new Gson()
                .fromJson(
                        "[{\"name\":\"Fresh\",\"group\":\"New\",\"kind\":\"teleop\",\"where\":\"org.example.FreshTeleOp\"}]",
                        JsonArray.class);
        SimCatalog.Entry entry = SimCatalog.fromJson(json).find("Fresh").get();
        assertEquals("Fresh", entry.name);
        assertEquals("New", entry.group);
        assertEquals("teleop", entry.kind);
        assertEquals("org.example.FreshTeleOp", entry.where);
        try {
            entry.opMode();
            fail("an entry from another JVM cannot be built here");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("Fresh"));
        }
    }
}
