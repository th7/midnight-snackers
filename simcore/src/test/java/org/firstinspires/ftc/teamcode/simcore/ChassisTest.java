package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ChassisTest {
    private static final double HALF = Chassis.SIZE_IN / 2;
    private static final Length POLLEN = Valid.length(2.5);
    private static final double TOUCHING = HALF + 2.5;
    private static final Heading AHEAD = Valid.heading(0);
    private static final Heading LEFT = Valid.heading(Math.PI / 2);
    private static final double DELTA = 1e-9;

    @Test
    public void aBallTouchingTheFrontFaceIsAgainstIt() {
        assertTrue(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING, 0), POLLEN));
    }

    @Test
    public void theIntakeReachesAQuarterInchPastTheFrontAndNoFurther() {
        assertTrue(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING + Chassis.INTAKE_REACH_IN - 0.01, 0), POLLEN));
        assertFalse(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING + Chassis.INTAKE_REACH_IN + 0.01, 0), POLLEN));
    }

    @Test
    public void theFrontIsTheRobotsWholeWidthAndNoWider() {
        assertTrue(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING, HALF - 0.01), POLLEN));
        assertTrue(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING, -(HALF - 0.01)), POLLEN));
        assertFalse(Chassis.againstTheFront(AHEAD, new Vec2(TOUCHING, HALF + 0.01), POLLEN));
    }

    @Test
    public void aBallBehindOrBesideTheRobotIsNotAgainstTheFront() {
        assertFalse(Chassis.againstTheFront(AHEAD, new Vec2(-TOUCHING, 0), POLLEN));
        assertFalse(Chassis.againstTheFront(AHEAD, new Vec2(0, TOUCHING), POLLEN));
    }

    @Test
    public void theFrontIsWhereTheRobotFaces() {
        assertTrue(Chassis.againstTheFront(LEFT, new Vec2(0, TOUCHING), POLLEN));
        assertFalse(Chassis.againstTheFront(LEFT, new Vec2(TOUCHING, 0), POLLEN));
    }

    @Test
    public void aBiggerBallIsAgainstTheFrontFartherOut() {
        Length nectar = Valid.length(3.5);

        assertTrue(Chassis.againstTheFront(AHEAD, new Vec2(HALF + 3.5, 0), nectar));
        assertFalse(Chassis.againstTheFront(AHEAD, new Vec2(HALF + 3.5, 0), POLLEN));
    }

    @Test
    public void whatTheFieldSeesTheRobotSeesTurnedByItsHeading() {
        Vec2 ahead = LEFT.onTheRobot(new Vec2(0, 1));
        assertEquals(1, ahead.x(), DELTA);
        assertEquals(0, ahead.y(), DELTA);

        Vec2 toItsLeft = LEFT.onTheRobot(new Vec2(-1, 0));
        assertEquals(0, toItsLeft.x(), DELTA);
        assertEquals(1, toItsLeft.y(), DELTA);

        Heading turned = Valid.heading(0.7);
        Vec2 there = new Vec2(3, -4);
        Vec2 back = turned.onTheField(turned.onTheRobot(there));
        assertEquals(there.x(), back.x(), DELTA);
        assertEquals(there.y(), back.y(), DELTA);
    }
}
