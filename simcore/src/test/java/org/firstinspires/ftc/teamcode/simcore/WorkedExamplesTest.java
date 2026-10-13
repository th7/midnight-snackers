package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.List;
import org.junit.Test;

public class WorkedExamplesTest {
    private static final ShotSettings DEFAULTS = ShotSettings.defaults();
    private static final Ball POLLEN = Valid.value(ShotSetup.pollen(DEFAULTS));
    private static final Surfaces SURFACES = Valid.value(ShotSetup.surfaces(DEFAULTS));
    private static final double ALPHA = POLLEN.inertiaFactor();
    private static final double R = POLLEN.radius();
    private static final double IN_PER_M = 1 / Length.METRES_PER_INCH;
    private static final double GRIPPING = ALPHA / (1 + ALPHA);

    private static Contacts.Body leaving(Vec3 velocity, Vec3 spin, Surface surface, Vec3 normal) {
        Contacts.Body ball = new Contacts.Body(Shot.LAUNCHED, POLLEN, velocity, spin);
        Contacts.Pressing press = new Contacts.Pressing(ball, new Contacts.Against.Fixed("it", surface), normal, true);
        return Contacts.resolve(SURFACES, List.of(press)).bodies().get(0);
    }

    private static double floorAngle(double spin) {
        double speed = 6 * IN_PER_M,
                along = speed * Math.cos(Math.toRadians(45)),
                down = speed * Math.sin(Math.toRadians(45));
        Contacts.Body after =
                leaving(new Vec3(along, 0, -down), new Vec3(0, spin, 0), Surface.FLOOR, new Vec3(0, 0, 1));
        return Math.toDegrees(Math.atan2(after.velocity().z(), after.velocity().x()));
    }

    private static double floorAngleByWalton(double spin) {
        double speed = 6 * IN_PER_M,
                along = speed * Math.cos(Math.toRadians(45)),
                down = speed * Math.sin(Math.toRadians(45));
        Bounce floor = SURFACES.of(Surface.FLOOR);
        double e = SURFACES.restitution(Surface.FLOOR, down);
        double slip = along - R * spin;
        double stick = (1 + floor.grip()) * GRIPPING * slip;
        double most = floor.friction() * (1 + e) * down;
        double across = Math.abs(stick) <= most ? stick : Math.signum(slip) * most;
        return Math.toDegrees(Math.atan2(e * down, along - across));
    }

    @Test
    public void onTheDefaultFloorBackspinKicksA45DegreeBounceUpTo63DegreesAndTopspinFlattensItTo37() {
        assertEquals(floorAngleByWalton(0), floorAngle(0), 1e-9);
        assertEquals(floorAngleByWalton(-60), floorAngle(-60), 1e-9);
        assertEquals(floorAngleByWalton(60), floorAngle(60), 1e-9);
        assertEquals(47, floorAngle(0), 0.5);
        assertEquals(63, floorAngle(-60), 0.5);
        assertEquals(37, floorAngle(60), 0.5);
    }

    @Test
    public void onTheDefaultWallAHeadOnHitAt5MetresASecondWith100RadiansOfSidespinGoesOff16AlongIt() {
        double speed = 5 * IN_PER_M;
        Contacts.Body after = leaving(new Vec3(speed, 0, 0), new Vec3(0, 0, 100), Surface.WALL, new Vec3(-1, 0, 0));

        Bounce wall = SURFACES.of(Surface.WALL);
        double slip = 100 * R;
        assertEquals(-(1 + wall.grip()) * GRIPPING * slip, after.velocity().y(), 1e-9);
        assertEquals(-1.6, after.velocity().y() / IN_PER_M, 0.05);
    }

    @Test
    public void sidespinOnlyGripsTheWallWhenTheHitIsHardEnoughToHoldIt() {
        double speed = 100;
        Contacts.Body after = leaving(new Vec3(speed, 0, 0), new Vec3(0, 0, 100), Surface.WALL, new Vec3(-1, 0, 0));

        Bounce wall = SURFACES.of(Surface.WALL);
        double most = wall.friction() * (1 + SURFACES.restitution(Surface.WALL, speed)) * speed;
        assertEquals(-most, after.velocity().y(), 1e-9);
    }

    private static double ofTheIdealSpeed(double kilogramSquareMetres) {
        WheelPair.Flywheel one = Valid.value(WheelPair.Flywheel.of(3.78, 1200));
        WheelPair.Flywheel two = Valid.value(WheelPair.Flywheel.of(3.78, 1600));
        WheelPair.Launched ideal = Valid.value(WheelPair.of(
                        POLLEN,
                        new Vec3(0, 0, 14),
                        0,
                        0,
                        0,
                        one,
                        two,
                        1,
                        1,
                        0,
                        new WheelPair.Inertia.KeepSpeed(),
                        new Vec2(0, 0)))
                .launch();
        WheelPair.Launched slowing = Valid.value(WheelPair.of(
                        POLLEN,
                        new Vec3(0, 0, 14),
                        0,
                        0,
                        0,
                        one,
                        two,
                        1,
                        1,
                        0,
                        Valid.value(WheelPair.Inertia.Slows.of(kilogramSquareMetres)),
                        new Vec2(0, 0)))
                .launch();
        return slowing.velocity().x() / ideal.velocity().x();
    }

    @Test
    public void aBareHogbackWheelThrowsAtFourFifthsOfItsSpeedAndOneWithASteelFlywheelAtNineTenths() {
        double hogback = 1.13e-4, steelFlywheel = 1.651e-4, radius = 3.78 * Length.METRES_PER_INCH / 2;
        double bare = hogback / (radius * radius), weighted = (hogback + steelFlywheel) / (radius * radius);
        double m = POLLEN.kilograms();

        assertEquals(2 * bare / (m + 2 * bare), ofTheIdealSpeed(hogback), 1e-9);
        assertEquals(2 * weighted / (m + 2 * weighted), ofTheIdealSpeed(hogback + steelFlywheel), 1e-9);
        assertEquals(0.80, ofTheIdealSpeed(hogback), 0.01);
        assertEquals(0.91, ofTheIdealSpeed(hogback + steelFlywheel), 0.01);
    }
}
