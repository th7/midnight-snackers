package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurnTest {
    private static final double DELTA = 1e-12;

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Vec3 turn(Turn turn, Vec3 v) {
        double w = turn.w(), x = turn.x(), y = turn.y(), z = turn.z();
        Vec3 u = new Vec3(x, y, z);
        Vec3 t = u.cross(v).times(2);
        return v.plus(t.times(w)).plus(u.cross(t));
    }

    private static void assertVec(Vec3 expected, Vec3 actual) {
        assertEquals("x of " + actual, expected.x(), actual.x(), DELTA);
        assertEquals("y of " + actual, expected.y(), actual.y(), DELTA);
        assertEquals("z of " + actual, expected.z(), actual.z(), DELTA);
    }

    @Test
    public void theIdentityTurnsNothing() {
        Turn identity = Turn.identity();

        assertEquals(1, identity.w(), 0);
        assertVec(new Vec3(1, 2, 3), turn(identity, new Vec3(1, 2, 3)));
    }

    @Test
    public void aSpinTurnsByItsRateForAsLongAsItLasts() {
        Turn quarter = Turn.identity().turnedBy(new Vec3(0, 0, Math.PI / 2), seconds(1));

        assertVec(new Vec3(0, 1, 0), turn(quarter, new Vec3(1, 0, 0)));
    }

    @Test
    public void turnsAddUpInTheFieldsFrame() {
        Turn first = Turn.identity().turnedBy(new Vec3(Math.PI / 2, 0, 0), seconds(1));
        Turn then = first.turnedBy(new Vec3(0, 0, Math.PI / 2), seconds(1));

        assertVec(new Vec3(1, 0, 0), turn(then, new Vec3(0, 0, 1)));
    }

    @Test
    public void noSpinIsNoTurn() {
        Turn turned = Turn.identity().turnedBy(Vec3.zero(), seconds(5));

        assertEquals(1, turned.w(), 0);
        assertEquals(0, turned.x(), 0);
    }

    @Test
    public void aTurnStaysAUnitQuaternionHoweverLongItSpins() {
        Turn spun = Turn.identity();
        for (int i = 0; i < 100000; i++) {
            spun = spun.turnedBy(new Vec3(130, -270, 410), seconds(1e-4));
        }

        double norm = spun.w() * spun.w() + spun.x() * spun.x() + spun.y() * spun.y() + spun.z() * spun.z();
        assertEquals(1, norm, 1e-12);
    }
}
