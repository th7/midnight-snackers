package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.List;
import java.util.Set;
import org.junit.Test;

public class IslandsTest {
    private static final double R = 1.4;

    private static Islands.Node node(String name, Islands.State state, double x) {
        return new Islands.Node(name, state, new Vec3(x, 0, R), R, Set.of());
    }

    @Test
    public void aMovingBallWakesEveryPieceItTouchesAndEveryPieceThoseTouch() {
        List<Islands.Node> row = List.of(
                node("ball", Islands.State.MOVING, 0),
                node("one", Islands.State.ASLEEP, 2 * R),
                node("two", Islands.State.ASLEEP, 4 * R),
                node("apart", Islands.State.ASLEEP, 9 * R));

        assertEquals(Set.of("one", "two"), Islands.toWake(row, 1e-6));
    }

    @Test
    public void aWakePassesThroughAPieceThatIsAwakeButResting() {
        List<Islands.Node> row = List.of(
                node("ball", Islands.State.MOVING, 0),
                node("resting", Islands.State.RESTING, 2 * R),
                node("behind", Islands.State.ASLEEP, 4 * R));

        assertEquals(Set.of("behind"), Islands.toWake(row, 1e-6));
    }

    @Test
    public void aBallAtRestWakesNothingSoPiecesRestingTogetherStayAsleep() {
        List<Islands.Node> pile = List.of(
                node("ball", Islands.State.RESTING, 0),
                node("one", Islands.State.ASLEEP, 2 * R),
                node("two", Islands.State.ASLEEP, 4 * R));

        assertEquals(Set.of(), Islands.toWake(pile, 1e-6));
    }

    @Test
    public void aPieceHeldInItsBoreNeitherMovesNorPassesAWakeOn() {
        List<Islands.Node> row = List.of(
                node("ball", Islands.State.MOVING, 0),
                node("held", Islands.State.HELD, 2 * R),
                node("behind", Islands.State.ASLEEP, 4 * R));

        assertEquals(Set.of(), Islands.toWake(row, 1e-6));
    }

    @Test
    public void aPieceAMovingBallWasHoldingUpWakesThoughTheBallHasMovedAway() {
        List<Islands.Node> lifted = List.of(
                new Islands.Node("ball", Islands.State.MOVING, new Vec3(0, 0, R), R, Set.of("above")),
                node("above", Islands.State.ASLEEP, 10 * R));

        assertEquals(Set.of("above"), Islands.toWake(lifted, 1e-6));
    }
}
