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
import org.firstinspires.ftc.nugget.NuggetTeleOp;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.opmode.AutoOp;
import org.firstinspires.ftc.teamcode.opmode.PlanOpModes;
import org.firstinspires.ftc.teamcode.opmode.RedTeleOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Test;

public class SimCatalogTest {
    private final SimCatalog catalog = SimCatalog.discover(TeamRobot.REGINALD);

    @Test
    public void nuggetsCatalogListsNuggetsOpModesAndNoneOfReginalds() {
        SimCatalog nuggets = SimCatalog.discover(TeamRobot.NUGGET);

        SimCatalog.Entry teleOp = nuggets.find("Nugget TeleOp").get();
        assertEquals("teleop", teleOp.kind);
        assertEquals("Nugget", teleOp.group);
        assertEquals(NuggetTeleOp.class.getName(), teleOp.where);
        assertNull("Nugget plays for no alliance", teleOp.alliance);
        assertTrue(teleOp.opMode() instanceof NuggetTeleOp);
        assertNotSame(teleOp.opMode(), teleOp.opMode());
        assertFalse(nuggets.find("driveForward").isPresent());
        assertFalse(nuggets.find("RedTeleOp").isPresent());
    }

    @Test
    public void reginaldsCatalogListsNoneOfNuggets() {
        assertFalse(catalog.find("Nugget TeleOp").isPresent());
        for (SimCatalog.Entry entry : catalog.entries()) {
            assertFalse(entry.where, entry.where.startsWith(SimCatalog.packageOf(TeamRobot.NUGGET) + "."));
        }
    }

    @Test
    public void eachRobotsOpModesAreItsOwnPackages() {
        assertEquals("org.firstinspires.ftc.teamcode", SimCatalog.packageOf(TeamRobot.REGINALD));
        assertEquals("org.firstinspires.ftc.nugget", SimCatalog.packageOf(TeamRobot.NUGGET));
    }

    @Test
    public void aFixedCatalogListsNuggetsOpModesTooWhenItIsGivenThem() {
        SimCatalog fixed = SimCatalog.of(NuggetTeleOp.class);

        assertTrue(fixed.find("Nugget TeleOp").get().opMode() instanceof NuggetTeleOp);
        assertEquals(List.of(NuggetTeleOp.class.getName()), fixed.sources());
    }

    @Test
    public void listsEveryAutoThePlanRegistrarRegisters() {
        SimCatalog.Entry forward = catalog.find("driveForward").get();

        assertEquals("auto", forward.kind);
        assertEquals("Autonomous", forward.group);
        assertEquals("Plans.driveForward()", forward.where);
        assertTrue(catalog.find("BlueScoreAThingFromBack").isPresent());
        assertTrue(catalog.find("RedScoreAThingFromBack").isPresent());
        assertTrue(
                "discovered, so another JVM lists the same without being told where to look",
                catalog.sources().isEmpty());
    }

    @Test
    public void listsEveryAnnotatedTeleOpToo() {
        SimCatalog.Entry red = catalog.find("RedTeleOp").get();

        assertEquals("teleop", red.kind);
        assertEquals("TeleOp", red.group);
        assertEquals(RedTeleOp.class.getName(), red.where);
        assertTrue(catalog.find("BlueTeleOp").isPresent());
        assertFalse(
                "Road Runner's tuning op modes are not ours to simulate",
                catalog.find("LocalizationTest").isPresent());
        assertFalse(
                "nested test op modes never reach the bench",
                catalog.find("Count to three").isPresent());
        assertFalse(
                "nested test op modes never reach the bench",
                catalog.find("Stick").isPresent());
    }

    @Test
    public void autosComeBeforeTeleOpsEachSortedByName() {
        List<SimCatalog.Entry> entries = catalog.entries();

        int firstTeleOp = entries.size();
        for (int i = 0; i < entries.size(); i++) {
            if (entries.get(i).kind.equals("teleop")) {
                firstTeleOp = Math.min(firstTeleOp, i);
            } else {
                assertTrue("auto " + entries.get(i).name + " listed after a TeleOp", i < firstTeleOp);
            }
        }
        assertTrue(firstTeleOp < entries.size());
        for (int i = 1; i < entries.size(); i++) {
            if (entries.get(i - 1).kind.equals(entries.get(i).kind)) {
                assertTrue(entries.get(i - 1).name.compareTo(entries.get(i).name) <= 0);
            }
        }
    }

    @Test
    public void aRegisteredInstanceIsTheOpModeEveryTimeAsOnTheRobotAndAnAnnotatedClassIsBuiltAfresh() {
        SimCatalog.Entry plan = catalog.find("driveForward").get();
        SimCatalog.Entry teleOp = catalog.find("RedTeleOp").get();

        assertSame(plan.opMode(), plan.opMode());
        assertTrue(plan.opMode() instanceof AutoOp);
        assertNotSame(teleOp.opMode(), teleOp.opMode());
        assertTrue(teleOp.opMode() instanceof RedTeleOp);
        assertFalse(catalog.find("org.example.Nope").isPresent());
    }

    @Test
    public void aFixedCatalogListsTheOpModesAndRegistrarsItIsGivenAndRemembersThem() {
        SimCatalog fixed = SimCatalog.of(StickTeleOp.class, ThreeLoopAuto.class, PlanOpModes.class);

        SimCatalog.Entry count = fixed.find("Count to three").get();
        assertEquals("auto", count.kind);
        assertEquals(ThreeLoopAuto.class.getName(), count.where);
        assertEquals("Plans.driveForward()", fixed.find("driveForward").get().where);
        SimCatalog.Entry stick = fixed.entries().get(fixed.entries().size() - 1);
        assertEquals("Stick", stick.name);
        assertEquals("teleop", stick.kind);
        assertEquals("Test", stick.group);
        assertEquals(
                List.of(StickTeleOp.class.getName(), ThreeLoopAuto.class.getName(), PlanOpModes.class.getName()),
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
    public static class AnotherCountToThree extends AutoOp {
        public AnotherCountToThree() {
            super(Alliance.RELATIVE);
        }

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
        assertEquals("Blue", catalog.find("BlueTeleOp").get().alliance);
        assertEquals("Red", catalog.find("RedTeleOp").get().alliance);
        assertEquals("Blue", catalog.find("BlueScoreAThingFromBack").get().alliance);
        assertEquals("Red", catalog.find("RedScoreAThingFromBack").get().alliance);
        assertNull(
                "a plan relative to where the robot starts plays for nobody",
                catalog.find("driveForward").get().alliance);

        SimCatalog parsed = SimCatalog.fromJson(catalog.toJson());

        assertEquals("Red", parsed.find("RedTeleOp").get().alliance);
        assertEquals("Blue", parsed.find("BlueScoreAThingFromBack").get().alliance);
        assertNull(parsed.find("driveForward").get().alliance);
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

        assertEquals("teleop", parsed.find("RedTeleOp").get().kind);
        assertEquals("Plans.driveForward()", parsed.find("driveForward").get().where);
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
