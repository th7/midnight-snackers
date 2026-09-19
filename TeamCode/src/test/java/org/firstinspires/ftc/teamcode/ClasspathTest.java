package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.SimCatalog;
import org.junit.Test;

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

    @Test
    public void everyClassInTheTeamCodePackageLoadsOnThisJvm() {
        Classpath.Scan scan = Classpath.scan(SimCatalog.TEAMCODE_PACKAGE);

        assertEquals(Map.of(), scan.unreadable);
        assertTrue(scan.classes.size() + " classes", scan.classes.size() > 50);
    }
}
