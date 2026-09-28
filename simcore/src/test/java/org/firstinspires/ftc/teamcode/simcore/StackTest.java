package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class StackTest {
    private static final Field.Flower FLOWER =
            Valid.value(Field.Flower.of("Flower", new Vec2(10, 0), 2, 2.4, 4.25, 0.35));
    private static final Length POLLEN = Valid.length(2.5);
    private static final Length SMALL = Valid.length(1.5);
    private static final List<Rolling<String>> NOTHING_ROLLING = List.of();
    private static final Seconds STEP = Valid.value(Seconds.of(0.005));
    private static final double DELTA = 1e-9;

    private static double heightOf(Stack<String> stack, String ball) {
        Optional<Vec3> at = stack.at(ball);
        assertTrue(ball + " is in the stack", at.isPresent());
        assertEquals(FLOWER.axis().x(), at.orElse(Vec3.zero()).x(), 0);
        assertEquals(FLOWER.axis().y(), at.orElse(Vec3.zero()).y(), 0);
        return at.orElse(Vec3.zero()).z();
    }

    private static Stack<String> after(Stack<String> stack, double seconds, List<Rolling<String>> rolling) {
        for (int i = 0; i < Math.round(seconds / STEP.value()); i++) {
            Stack.Settled<String> settled = stack.after(STEP, rolling);
            assertEquals(List.of(), settled.leaving());
            stack = settled.stack();
        }
        return stack;
    }

    private static Stack<String> threeRested() {
        Stack.Settled<String> rested = Stack.<String>in(FLOWER)
                .with("top", POLLEN, 30)
                .with("bottom", POLLEN, 10)
                .with("middle", POLLEN, 20)
                .rested(NOTHING_ROLLING);
        assertEquals(List.of(), rested.leaving());
        return rested.stack();
    }

    @Test
    public void aRestedStackStandsEachBallOnTheOneUnderItFromTheFloorUp() {
        Stack<String> stack = threeRested();

        assertEquals(2.5, heightOf(stack, "bottom"), DELTA);
        assertEquals(7.5, heightOf(stack, "middle"), DELTA);
        assertEquals(12.5, heightOf(stack, "top"), DELTA);
        assertEquals(3, stack.size());
    }

    @Test
    public void aStackThatStandsStaysPut() {
        Stack<String> stack = after(threeRested(), 2, NOTHING_ROLLING);

        assertEquals(2.5, heightOf(stack, "bottom"), 0);
        assertEquals(7.5, heightOf(stack, "middle"), 0);
        assertEquals(12.5, heightOf(stack, "top"), 0);
    }

    @Test
    public void aBallWithNothingUnderItFallsUnderGravity() {
        Stack<String> stack = Stack.<String>in(FLOWER).with("ball", POLLEN, 20);

        Stack<String> fallen = stack.after(STEP, NOTHING_ROLLING).stack();

        double speed = Flight.GRAVITY_IN_PER_S2 * STEP.value();
        assertEquals(20 - speed * STEP.value(), heightOf(fallen, "ball"), DELTA);
        assertEquals(
                "and faster the next step",
                20 - 3 * speed * STEP.value(),
                heightOf(fallen.after(STEP, NOTHING_ROLLING).stack(), "ball"),
                DELTA);
    }

    @Test
    public void aBallThatLandsSlowlyStandsWhereItLands() {
        Stack<String> stack = Stack.<String>in(FLOWER).with("ball", POLLEN, 2.5 + 1e-6);

        Stack<String> landed = stack.after(STEP, NOTHING_ROLLING).stack();

        assertEquals(2.5, heightOf(landed, "ball"), 0);
        assertEquals(2.5, heightOf(after(landed, 0.5, NOTHING_ROLLING), "ball"), 0);
    }

    @Test
    public void aBallThatLandsHardBouncesBeforeItStands() {
        Stack<String> stack = Stack.<String>in(FLOWER).with("ball", POLLEN, 20);

        boolean landed = false;
        double highestAfter = 0;
        for (int i = 0; i < 400; i++) {
            stack = stack.after(STEP, NOTHING_ROLLING).stack();
            double z = heightOf(stack, "ball");
            if (landed) {
                highestAfter = Math.max(highestAfter, z);
            }
            landed = landed || z == 2.5;
        }

        assertTrue("it came down", landed);
        assertTrue("and went up again, to " + highestAfter, highestAfter > 3);
        assertEquals("and stands", 2.5, heightOf(stack, "ball"), 0);
    }

    @Test
    public void aBallThatComesToRestBelowTheLipLeavesTheBoreAndWhatIsOnItStandsOnIt() {
        Stack.Settled<String> rested = Stack.<String>in(FLOWER)
                .with("small", SMALL, 5)
                .with("pollen", POLLEN, 20)
                .rested(NOTHING_ROLLING);

        assertEquals(List.of("small"), rested.leaving());
        assertEquals(Optional.empty(), rested.stack().at("small"));
        assertEquals(1, rested.stack().size());
        assertEquals(3 + 2.5, heightOf(rested.stack(), "pollen"), DELTA);
    }

    @Test
    public void aBallThatFallsToRestBelowTheLipLeavesTheBore() {
        Stack<String> stack = Stack.<String>in(FLOWER).with("small", SMALL, 1.5 + 1e-6);

        Stack.Settled<String> settled = stack.after(STEP, NOTHING_ROLLING);

        assertEquals(List.of("small"), settled.leaving());
        assertEquals(0, settled.stack().size());
    }

    @Test
    public void theStackStandsOnABallRollingInItsBoreAndNotOnOneBesideIt() {
        List<Rolling<String>> inTheBore = List.of(new Rolling<>("rolling", new Vec2(10.5, 0), POLLEN));
        List<Rolling<String>> beside = List.of(new Rolling<>("rolling", new Vec2(13, 0), POLLEN));
        Stack<String> stack = Stack.<String>in(FLOWER).with("ball", POLLEN, 20);

        assertEquals(5 + 2.5, heightOf(stack.rested(inTheBore).stack(), "ball"), DELTA);
        assertEquals(2.5, heightOf(stack.rested(beside).stack(), "ball"), DELTA);
    }

    @Test
    public void takingTheBottomBallOutBringsTheStackDownOnePlaceWhereItStandsAgain() {
        Stack<String> stack = threeRested().without("bottom");

        Stack<String> down = after(stack, 2, NOTHING_ROLLING);

        assertEquals(Optional.empty(), down.at("bottom"));
        assertEquals(2.5, heightOf(down, "middle"), 0);
        assertEquals(7.5, heightOf(down, "top"), 0);
    }

    @Test
    public void aBallGoesIntoTheStackInOrderOfHeight() {
        Stack<String> stack = Stack.<String>in(FLOWER)
                .with("high", POLLEN, 30)
                .with("low", POLLEN, 10)
                .with("level", POLLEN, 10);

        assertEquals(List.of("level", "low", "high"), stack.fromTheBottom());
        assertEquals(FLOWER, stack.flower());
        assertEquals(
                stack.fromTheBottom(), stack.without("nothing of the stack's").fromTheBottom());
    }
}
