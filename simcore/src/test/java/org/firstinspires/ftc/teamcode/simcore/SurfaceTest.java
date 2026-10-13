package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.HashSet;
import java.util.Set;
import org.junit.Test;

public class SurfaceTest {
    @Test
    public void everySurfaceSaysWhatItIsMadeOf() {
        for (Surface surface : Surface.values()) {
            assertFalse(surface + " has a label", surface.label().isBlank());
            assertFalse(surface + " says what it is", surface.says().isBlank());
        }
    }

    @Test
    public void aSurfaceIsAskedForByItsLowerCaseName() {
        Set<String> asked = new HashSet<>();
        for (Surface surface : Surface.values()) {
            assertEquals(surface.name().toLowerCase(java.util.Locale.ROOT), surface.asked());
            assertTrue(asked.add(surface.asked()));
        }
    }

    @Test
    public void aBallMeetsFiveSurfacesOfTheFieldAndTheOtherBalls() {
        assertEquals(6, Surface.values().length);
        assertEquals(Surface.PIECE, Surface.valueOf("PIECE"));
    }
}
