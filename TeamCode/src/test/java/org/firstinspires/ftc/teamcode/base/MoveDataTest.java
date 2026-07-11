package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class MoveDataTest {
    private static final float DELTA = 0.0001f;

    private void assertPowers(MoveData moveData, float frontLeft, float frontRight, float rearLeft, float rearRight) {
        assertEquals(frontLeft, moveData.frontLeftPower(), DELTA);
        assertEquals(frontRight, moveData.frontRightPower(), DELTA);
        assertEquals(rearLeft, moveData.rearLeftPower(), DELTA);
        assertEquals(rearRight, moveData.rearRightPower(), DELTA);
    }

    @Test
    public void straightDrivesAllWheelsTheSame() {
        assertPowers(MoveData.straight(0.5f, 0f, 1f), 0.5f, 0.5f, 0.5f, 0.5f);
    }

    @Test
    public void turnSpinsTheSidesOppositeWays() {
        assertPowers(MoveData.turn(0.5f, 0f, 1f), -0.5f, 0.5f, -0.5f, 0.5f);
    }

    @Test
    public void strafeCrossesTheDiagonals() {
        assertPowers(MoveData.strafe(0.5f, 0f, 1f), -0.5f, 0.5f, 0.5f, -0.5f);
    }

    @Test
    public void minPushesSmallPowersUpToTheMinimum() {
        assertPowers(MoveData.straight(0.05f, 0.1f, 1f), 0.1f, 0.1f, 0.1f, 0.1f);
        assertPowers(MoveData.straight(-0.05f, 0.1f, 1f), -0.1f, -0.1f, -0.1f, -0.1f);
    }

    @Test
    public void zeroStaysZeroDespiteTheMinimum() {
        assertPowers(MoveData.straight(0f, 0.1f, 1f), 0f, 0f, 0f, 0f);
    }

    @Test
    public void maxCapsPowerInBothDirections() {
        assertPowers(MoveData.straight(1.5f, 0f, 1f), 1f, 1f, 1f, 1f);
        assertPowers(MoveData.straight(-1.5f, 0f, 1f), -1f, -1f, -1f, -1f);
    }

    @Test
    public void addSumsPerWheelAndClampsToFullPower() {
        MoveData combined = MoveData.straight(0.8f, 0f, 1f).add(MoveData.turn(0.8f, 0f, 1f));

        assertPowers(combined, 0f, 1f, 0f, 1f);
    }
}
