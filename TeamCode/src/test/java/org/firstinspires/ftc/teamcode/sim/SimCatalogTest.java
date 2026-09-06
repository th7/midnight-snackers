package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.auto.DriveForward;
import org.firstinspires.ftc.teamcode.auto.ForwardLeftBackwardRight;
import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.junit.Test;

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
    }

    @Test
    public void namesComeFromTheAutonomousAnnotationAndEntriesAreSortedByName() {
        List<SimCatalog.Entry> entries = catalog.entries();

        SimCatalog.Entry forward = entries.stream().filter(e -> e.type == DriveForward.class).findFirst().get();
        assertEquals("DriveForward", forward.name);
        assertEquals("Autonomous", forward.group);
        for (int i = 1; i < entries.size(); i++) {
            assertTrue(entries.get(i - 1).name.compareTo(entries.get(i).name) <= 0);
        }
    }

    @Test
    public void looksUpAnEntryByClassNameAndBuildsAFreshOpMode() throws Exception {
        Optional<SimCatalog.Entry> entry = catalog.find(ForwardLeftBackwardRight.class.getName());

        assertTrue(entry.isPresent());
        AutoOp first = entry.get().create();
        AutoOp second = entry.get().create();
        assertTrue(first instanceof ForwardLeftBackwardRight);
        assertTrue(first != second);
        assertFalse(catalog.find("org.example.Nope").isPresent());
    }
}
