package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;
import org.junit.Test;

public class DrawsTest {
    private static final long[] SEEDS = {0, 1, 4, 7, 42, -3, Long.MAX_VALUE, Long.MIN_VALUE};

    @Test
    public void theDrawsAreJavaUtilRandomsSoASeedDrawsTheRobotItAlwaysHas() {
        for (long seed : SEEDS) {
            Random random = new Random(seed);
            Draws draws = Draws.seeded(seed);
            for (int i = 0; i < 3000; i++) {
                boolean gaussian = i % 5 != 0 && i % 7 != 0;
                Drawn<Double> drawn = gaussian ? draws.nextGaussian() : draws.nextDouble();
                double expected = gaussian ? random.nextGaussian() : random.nextDouble();
                assertEquals("seed " + seed + ", draw " + i, expected, drawn.value(), 0);
                draws = drawn.next();
            }
        }
    }

    @Test
    public void drawingLeavesTheDrawsItWasMadeFromAsTheyWere() {
        Draws draws = Draws.seeded(9).nextGaussian().next();

        assertEquals(draws.nextGaussian().value(), draws.nextGaussian().value(), 0);
        assertEquals(draws.nextDouble().value(), draws.nextDouble().value(), 0);
    }

    @Test
    public void aDoubleIsAtLeastZeroAndBelowOne() {
        Draws draws = Draws.seeded(3);
        for (int i = 0; i < 10_000; i++) {
            Drawn<Double> drawn = draws.nextDouble();
            assertTrue("draw " + i + ": " + drawn.value(), drawn.value() >= 0 && drawn.value() < 1);
            draws = drawn.next();
        }
    }
}
