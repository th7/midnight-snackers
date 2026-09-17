package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.SimCatalog;
import org.junit.Test;

/**
 * A scan of the classpath is what the invariant tests look at. One that quietly left out what it
 * could not read would let those tests pass by looking at less, so a scan says what it could not
 * read and the classes are only handed over when there is nothing to say.
 */
public class ClasspathTest {
    @Test
    public void aScanThatCouldNotReadEveryClassJudgesNothing() {
        Classpath.Scan scan = new Classpath.Scan(
                List.of(Classpath.class), Map.of("org.firstinspires.ftc.teamcode.Ghost", "NoClassDefFoundError"));

        IllegalStateException refused = assertThrows(IllegalStateException.class, scan::all);

        assertTrue(refused.getMessage(), refused.getMessage().contains("org.firstinspires.ftc.teamcode.Ghost"));
        assertTrue(refused.getMessage(), refused.getMessage().contains("NoClassDefFoundError"));
    }

    @Test
    public void aScanThatReadEverythingHandsOverItsClasses() {
        Classpath.Scan scan = new Classpath.Scan(List.of(Classpath.class), Map.of());

        assertEquals(List.of(Classpath.class), scan.all());
    }

    /**
     * And the real one reads everything. A class of ours that will not load on a plain JVM -- one
     * that touches Android, usually -- would be dropped from the scan, and the tests that check
     * every subsystem is ticked and every op mode is listed would pass having never seen it.
     */
    @Test
    public void everyClassInTheTeamCodePackageLoadsOnThisJvm() {
        Classpath.Scan scan = Classpath.scan(SimCatalog.TEAMCODE_PACKAGE);

        assertEquals(Map.of(), scan.unreadable);
        assertTrue(scan.classes.size() + " classes", scan.classes.size() > 50);
    }
}
