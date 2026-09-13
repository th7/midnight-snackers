package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.auto.DriveForward;
import org.firstinspires.ftc.teamcode.auto.ForwardLeftBackwardRight;
import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.base.BlueTeleOp;
import org.firstinspires.ftc.teamcode.base.OpMode;
import org.firstinspires.ftc.teamcode.base.RedTeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.LocalizationTest;
import org.junit.Test;

import com.google.gson.Gson;
import com.google.gson.JsonArray;

import java.util.List;
import java.util.Optional;

public class SimCatalogTest {
    private final SimCatalog catalog = SimCatalog.discover();

    @Test
    public void findsEveryRegisteredAutonomousOpMode() {
        List<SimCatalog.Entry> entries = catalog.entries();

        assertTrue(entries.stream().anyMatch(e -> e.type == ForwardLeftBackwardRight.class));
        assertTrue(entries.stream().anyMatch(e -> e.type == DriveForward.class));
        assertFalse("abstract bases are not runnable", entries.stream().anyMatch(e -> e.type == AutoOp.class));
        assertEquals("auto", find(DriveForward.class).kind);
    }

    @Test
    public void findsEveryRegisteredTeleOpToo() {
        List<SimCatalog.Entry> entries = catalog.entries();

        assertTrue(entries.stream().anyMatch(e -> e.type == RedTeleOp.class));
        assertTrue(entries.stream().anyMatch(e -> e.type == BlueTeleOp.class));
        assertEquals("teleop", find(RedTeleOp.class).kind);
        assertEquals("RedTeleOp", find(RedTeleOp.class).name);
        assertEquals("TeleOp", find(RedTeleOp.class).group);
        assertFalse("Road Runner's tuning op modes are not ours to simulate",
                entries.stream().anyMatch(e -> e.className.equals(LocalizationTest.class.getName())));
        assertFalse("abstract bases are not runnable",
                entries.stream().anyMatch(e -> e.type == org.firstinspires.ftc.teamcode.TeleOp.class));
        assertFalse("nested test op modes never reach the bench",
                entries.stream().anyMatch(e -> e.type == TestTeleOps.StickTeleOp.class));
    }

    @Test
    public void namesComeFromTheAnnotationAndAutosComeBeforeTeleOpsEachSortedByName() {
        List<SimCatalog.Entry> entries = catalog.entries();

        SimCatalog.Entry forward = find(DriveForward.class);
        assertEquals("DriveForward", forward.name);
        assertEquals("Autonomous", forward.group);
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
    public void looksUpAnEntryByClassNameAndBuildsAFreshOpMode() throws Exception {
        Optional<SimCatalog.Entry> entry = catalog.find(ForwardLeftBackwardRight.class.getName());

        assertTrue(entry.isPresent());
        OpMode first = entry.get().create();
        OpMode second = entry.get().create();
        assertTrue(first instanceof ForwardLeftBackwardRight);
        assertTrue(first != second);
        assertTrue(catalog.find(RedTeleOp.class.getName()).get().create() instanceof RedTeleOp);
        assertFalse(catalog.find("org.example.Nope").isPresent());
        assertEquals(ForwardLeftBackwardRight.class.getName(), entry.get().className);
    }

    @Test
    public void theJsonListingCarriesTheKindAndParsesBackWithoutATypeToBuild() {
        JsonArray listed = catalog.toJson();
        SimCatalog parsed = SimCatalog.fromJson(listed);

        assertEquals("teleop", parsed.find(RedTeleOp.class.getName()).get().kind);
        assertEquals("auto", parsed.find(DriveForward.class.getName()).get().kind);

        JsonArray json = new Gson().fromJson("[{\"name\":\"Fresh\",\"group\":\"New\",\"kind\":\"teleop\",\"opMode\":\"org.example.FreshTeleOp\"}]", JsonArray.class);
        SimCatalog.Entry entry = SimCatalog.fromJson(json).find("org.example.FreshTeleOp").get();
        assertEquals("Fresh", entry.name);
        assertEquals("New", entry.group);
        assertEquals("teleop", entry.kind);
        assertEquals("org.example.FreshTeleOp", entry.className);
        assertEquals(null, entry.type);
        try {
            entry.create();
            assertTrue("an entry from another JVM cannot be built here", false);
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("org.example.FreshTeleOp"));
        }
    }

    @Test
    public void aFixedCatalogKnowsTheKindOfEachOpMode() {
        SimCatalog fixed = SimCatalog.of(TestTeleOps.StickTeleOp.class, TestAutos.ThreeLoopAuto.class);

        assertEquals("auto", fixed.entries().get(0).kind);
        assertEquals("Count to three", fixed.entries().get(0).name);
        assertEquals("teleop", fixed.entries().get(1).kind);
        assertEquals("Stick", fixed.entries().get(1).name);
        assertEquals("Test", fixed.entries().get(1).group);
    }

    private SimCatalog.Entry find(Class<?> type) {
        return catalog.entries().stream().filter(e -> e.type == type).findFirst().get();
    }
}
