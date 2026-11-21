package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.Test;

public class KeptModelsTest {
    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    @Test
    public void onWindowsTheyAreKeptUnderLocalAppData() {
        KeptModels kept = Valid.value(KeptModels.of(
                "Windows 11", Map.of("LOCALAPPDATA", "C:\\Users\\ada\\AppData\\Local"), "C:\\Users\\ada"));

        assertEquals("C:\\Users\\ada\\AppData\\Local", kept.root());
        assertEquals(List.of("midnight-snackers", "field-models"), kept.under());
    }

    @Test
    public void onWindowsWithNoLocalAppDataTheyAreRefusedAPlaceNamingIt() {
        String said = rejection(KeptModels.of("Windows 11", Map.of(), "C:\\Users\\ada"));

        assertTrue(said, said.contains("LOCALAPPDATA"));
    }

    @Test
    public void onAMacTheyAreKeptUnderApplicationSupportWhichMacOsNeverPurges() {
        KeptModels kept = Valid.value(KeptModels.of("Mac OS X", Map.of("HOME", "/Users/ada"), "/Users/other"));

        assertEquals("/Users/ada", kept.root());
        assertEquals(List.of("Library", "Application Support", "midnight-snackers", "field-models"), kept.under());
    }

    @Test
    public void elsewhereTheyAreKeptUnderTheXdgDataHome() {
        KeptModels kept = Valid.value(KeptModels.of("Linux", Map.of("XDG_DATA_HOME", "/data/ada"), "/home/ada"));

        assertEquals("/data/ada", kept.root());
        assertEquals(List.of("midnight-snackers", "field-models"), kept.under());
    }

    @Test
    public void elsewhereWithNoAbsoluteXdgDataHomeTheyAreKeptUnderTheHomesLocalShare() {
        KeptModels kept = Valid.value(KeptModels.of("Linux", Map.of("XDG_DATA_HOME", "relative"), "/home/ada"));

        assertEquals("/home/ada", kept.root());
        assertEquals(List.of(".local", "share", "midnight-snackers", "field-models"), kept.under());
    }

    @Test
    public void withNoHomeAtAllTheyAreRefusedAPlace() {
        String said = rejection(KeptModels.of("Linux", Map.of(), " "));

        assertTrue(said, said.contains("HOME"));
    }
}
