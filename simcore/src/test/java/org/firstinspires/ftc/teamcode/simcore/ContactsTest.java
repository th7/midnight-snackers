package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.Test;

public class ContactsTest {
    private static final Vec3 UP = new Vec3(0, 0, 1);
    private static final Vec3 OFF_THE_WALL = new Vec3(1, 0, 0);
    private static final Contacts.Against FLOOR = new Contacts.Against.Fixed("the floor", Surface.FLOOR);
    private static final Contacts.Against WALL = new Contacts.Against.Fixed("the audience's wall", Surface.WALL);

    private static Pollen pollen(double inertiaFactor) {
        return Valid.value(Pollen.of(2.8, 24.9, inertiaFactor));
    }

    private static Bounce bounce(double restitution, double friction, double grip) {
        return Valid.value(Bounce.of(restitution, friction, grip));
    }

    private static Surfaces everywhere(Bounce bounce, double impactTwist, double spinTwist, double rollingResistance) {
        return Valid.value(Surfaces.of(
                bounce, bounce, bounce, bounce, bounce, bounce, 0, 4, impactTwist, spinTwist, rollingResistance));
    }

    private static Contacts.Body body(String name, Pollen shape, Vec3 velocity, Vec3 spin) {
        return new Contacts.Body(name, shape, velocity, spin);
    }

    private static Contacts.Resolved pressed(Surfaces surfaces, Contacts.Pressing... pressing) {
        return Contacts.resolve(surfaces, List.of(pressing));
    }

    private static Contacts.Body only(Contacts.Resolved resolved) {
        assertEquals(1, resolved.bodies().size());
        return resolved.bodies().get(0);
    }

    private static Contacts.Body named(String name, Contacts.Resolved resolved) {
        for (Contacts.Body body : resolved.bodies()) {
            if (body.name().equals(name)) {
                return body;
            }
        }
        throw new AssertionError(name + " is not among " + resolved.bodies());
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double delta) {
        assertEquals(what + ": x of " + actual + ", not " + expected, expected.x(), actual.x(), delta);
        assertEquals(what + ": y of " + actual + ", not " + expected, expected.y(), actual.y(), delta);
        assertEquals(what + ": z of " + actual + ", not " + expected, expected.z(), actual.z(), delta);
    }

    private static Vec3 across(Vec3 vector, Vec3 normal) {
        return vector.minus(normal.times(vector.dot(normal)));
    }

    private static double inertia(Pollen shape) {
        return shape.inertiaFactor() * shape.kilograms() * shape.radius() * shape.radius();
    }

    private static double energy(Contacts.Body body) {
        return 0.5 * body.shape().kilograms() * body.velocity().dot(body.velocity())
                + 0.5 * inertia(body.shape()) * body.spin().dot(body.spin());
    }

    private static Vec3 momentum(Contacts.Body body) {
        return body.velocity().times(body.shape().kilograms());
    }

    private static Vec3 angularMomentumAbout(Vec3 contactToCentre, Contacts.Body body) {
        return body.spin().times(inertia(body.shape())).plus(contactToCentre.cross(momentum(body)));
    }

    private static Vec3 contactVelocity(Contacts.Body body, Vec3 normal) {
        return body.velocity().plus(body.spin().cross(normal.times(-body.shape().radius())));
    }

    private record Walton(Vec3 velocity, Vec3 spin, double normalImpulse, boolean grips) {}

    private static Walton walton(
            Pollen shape, Vec3 velocity, Vec3 spin, Vec3 normal, double restitution, double friction, double grip) {
        double m = shape.kilograms(), r = shape.radius(), alpha = shape.inertiaFactor();
        Vec3 arm = normal.times(-r);
        Vec3 c = velocity.plus(spin.cross(arm));
        double vn = c.dot(normal);
        Vec3 ct = across(c, normal);
        double jn = -m * (1 + restitution) * vn;
        Vec3 stick = ct.times(-(1 + grip) * m * alpha / (1 + alpha));
        boolean grips = stick.length() <= friction * jn;
        Vec3 jt = grips ? stick : ct.times(-friction * jn / ct.length());
        Vec3 after = velocity.plus(normal.times(jn / m)).plus(jt.times(1 / m));
        Vec3 turned = spin.plus(arm.cross(jt).times(1 / (alpha * m * r * r)));
        return new Walton(after, turned, jn, grips);
    }

    private static Vec3 randomVec(Random random, double most) {
        return new Vec3(
                most * (2 * random.nextDouble() - 1),
                most * (2 * random.nextDouble() - 1),
                most * (2 * random.nextDouble() - 1));
    }

    private static Vec3 randomUnit(Random random) {
        return new Vec3(random.nextGaussian(), random.nextGaussian(), random.nextGaussian()).unit();
    }

    private static Pollen randomPollen(Random random) {
        return Valid.value(Pollen.of(
                2.6 + random.nextDouble(), 20 + 35 * random.nextDouble(), 0.4 + random.nextDouble() * (2.0 / 3 - 0.4)));
    }

    private static Bounce randomBounce(Random random) {
        return bounce(random.nextDouble(), 1.5 * random.nextDouble(), random.nextDouble());
    }

    private static Vec3 into(Vec3 velocity, Vec3 normal, double speed) {
        return velocity.plus(normal.times(-velocity.dot(normal) - speed));
    }

    @Test
    public void aFreshImpactThatCanGripIsWaltonsMapGripping() {
        Pollen shape = pollen(0.634);
        Vec3 normal = new Vec3(0.3, -0.2, 1).unit();
        Vec3 velocity = new Vec3(60, -30, -200), spin = new Vec3(5, -12, 3);
        Walton expected = walton(shape, velocity, spin, normal, 0.55, 0.5, 0.2);
        assertTrue("this impact can grip", expected.grips());

        Contacts.Resolved resolved = pressed(
                everywhere(bounce(0.55, 0.5, 0.2), 0, 0, 0.04),
                new Contacts.Pressing(body("pollen", shape, velocity, spin), FLOOR, normal, true));

        Contacts.Body after = only(resolved);
        assertEquals("pollen", after.name());
        assertEquals(shape, after.shape());
        assertVec("velocity", expected.velocity(), after.velocity(), 1e-9);
        assertVec("spin", expected.spin(), after.spin(), 1e-9);
        assertEquals(1, resolved.hits().size());
        Contacts.Hit hit = resolved.hits().get(0);
        assertEquals("pollen", hit.ball());
        assertEquals("the floor", hit.against());
        assertEquals(Surface.FLOOR, hit.surface());
        assertVec("normal", normal, hit.normal(), 1e-15);
        assertEquals(-velocity.dot(normal), hit.approach(), 1e-12);
        assertEquals(
                across(contactVelocity(body("pollen", shape, velocity, spin), normal), normal)
                        .length(),
                hit.slip(),
                1e-12);
        assertEquals(0.55, hit.restitution(), 0);
        assertTrue(hit.gripped());
        assertEquals(expected.normalImpulse(), hit.normalImpulse(), 1e-12);
    }

    @Test
    public void aFreshImpactThatCannotGripIsWaltonsMapSlidingAtTheFrictionLimit() {
        Pollen shape = pollen(0.634);
        Vec3 normal = new Vec3(0.3, -0.2, 1).unit();
        Vec3 velocity = new Vec3(300, 50, -150), spin = new Vec3(5, -12, 3);
        Walton expected = walton(shape, velocity, spin, normal, 0.55, 0.5, 0.2);
        assertFalse("this impact cannot grip", expected.grips());

        Contacts.Resolved resolved = pressed(
                everywhere(bounce(0.55, 0.5, 0.2), 0, 0, 0.04),
                new Contacts.Pressing(body("pollen", shape, velocity, spin), FLOOR, normal, true));

        Contacts.Body after = only(resolved);
        assertVec("velocity", expected.velocity(), after.velocity(), 1e-9);
        assertVec("spin", expected.spin(), after.spin(), 1e-9);
        Contacts.Hit hit = resolved.hits().get(0);
        assertFalse(hit.gripped());
        assertEquals(expected.normalImpulse(), hit.normalImpulse(), 1e-12);
    }

    @Test
    public void everyImpactOnAFixedSurfaceIsWaltonsMap() {
        Random random = new Random(1750);
        for (int i = 0; i < 2000; i++) {
            Pollen shape = randomPollen(random);
            Bounce bounce = randomBounce(random);
            Surfaces surfaces = everywhere(bounce, 0, 0, 0.2 * random.nextDouble());
            Vec3 normal = randomUnit(random);
            Vec3 velocity = into(randomVec(random, 300), normal, 5 + 300 * random.nextDouble());
            Vec3 spin = randomVec(random, 300);
            Walton expected =
                    walton(shape, velocity, spin, normal, bounce.restitution(), bounce.friction(), bounce.grip());

            Contacts.Body after = only(pressed(
                    surfaces, new Contacts.Pressing(body("pollen", shape, velocity, spin), FLOOR, normal, true)));

            assertVec("velocity of impact " + i, expected.velocity(), after.velocity(), 1e-9);
            assertVec("spin of impact " + i, expected.spin(), after.spin(), 1e-9);
        }
    }

    @Test
    public void overManyRandomImpactsEnergyNeverRisesAngularMomentumAboutTheContactIsKeptAndFrictionStaysInItsCone() {
        Random random = new Random(175);
        int gripped = 0, slid = 0;
        for (int i = 0; i < 10000; i++) {
            Pollen shape = randomPollen(random);
            double m = shape.kilograms(), r = shape.radius(), alpha = shape.inertiaFactor();
            Bounce bounce = randomBounce(random);
            double threshold = 0.5 + 24.5 * random.nextDouble();
            Surfaces surfaces = Valid.value(Surfaces.of(
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    0.2 * random.nextDouble(),
                    threshold,
                    0,
                    0,
                    0.2 * random.nextDouble()));
            Vec3 normal = randomUnit(random);
            double approach = threshold + 0.01 + 400 * random.nextDouble();
            Contacts.Body before =
                    body("pollen", shape, into(randomVec(random, 300), normal, approach), randomVec(random, 300));
            String impact = "impact " + i + " of " + before + " on " + normal + " with " + bounce;

            Contacts.Resolved resolved = pressed(surfaces, new Contacts.Pressing(before, FLOOR, normal, true));

            Contacts.Body after = only(resolved);
            assertEquals(impact, 1, resolved.hits().size());
            Contacts.Hit hit = resolved.hits().get(0);
            assertTrue(impact, energy(after) <= energy(before) * (1 + 1e-12));
            Vec3 arm = normal.times(r);
            double angularScale = inertia(shape) * before.spin().length()
                    + r * m * before.velocity().length();
            assertVec(
                    impact + ": angular momentum about the contact, across the normal",
                    across(angularMomentumAbout(arm, before), normal),
                    across(angularMomentumAbout(arm, after), normal),
                    1e-10 * angularScale);
            assertEquals(
                    impact + ": spin about the normal",
                    before.spin().dot(normal),
                    after.spin().dot(normal),
                    1e-10 * before.spin().length());
            Vec3 impulse = after.velocity().minus(before.velocity()).times(m);
            double impulseScale =
                    m * (before.velocity().length() + r * before.spin().length());
            double normalImpulse = impulse.dot(normal);
            assertEquals(impact, hit.normalImpulse(), normalImpulse, 1e-10 * impulseScale);
            assertEquals(impact, approach, hit.approach(), 1e-9 * approach);
            assertEquals(impact, surfaces.restitution(Surface.FLOOR, approach), hit.restitution(), 1e-12);
            assertEquals(impact, m * (1 + hit.restitution()) * approach, hit.normalImpulse(), 1e-10 * impulseScale);
            double rubbed = across(impulse, normal).length();
            assertTrue(
                    impact + ": " + rubbed + " outside the cone of " + bounce.friction() * hit.normalImpulse(),
                    rubbed <= bounce.friction() * hit.normalImpulse() + 1e-10 * impulseScale);
            double stick = (1 + bounce.grip()) * m * alpha / (1 + alpha) * hit.slip();
            double cone = bounce.friction() * hit.normalImpulse();
            if (stick < cone - 1e-9 * impulseScale) {
                assertTrue(impact + ": the stick impulse " + stick + " fits the cone " + cone, hit.gripped());
                gripped++;
            } else if (stick > cone + 1e-9 * impulseScale) {
                assertFalse(impact + ": the stick impulse " + stick + " is outside the cone " + cone, hit.gripped());
                slid++;
            }
        }
        assertTrue("gripped " + gripped, gripped > 1000);
        assertTrue("slid " + slid, slid > 1000);
    }

    @Test
    public void garwinsPerfectlyGrippingSolidBallLeavesAtThreeSeventhsAndSpinsAtTenSevenths() {
        Pollen solid = pollen(0.4);
        Surfaces superball = everywhere(bounce(1, 1.5, 1), 0, 0, 0);
        List<Vec3> normals = List.of(UP, OFF_THE_WALL, new Vec3(0, -1, 0), new Vec3(1, 1, 1).unit());
        for (Vec3 normal : normals) {
            Vec3 along = normal.cross(new Vec3(0.2, 0.7, -0.4)).unit();
            Vec3 velocity = along.times(100).plus(normal.times(-300));

            Contacts.Resolved resolved = pressed(
                    superball,
                    new Contacts.Pressing(body("pollen", solid, velocity, Vec3.zero()), FLOOR, normal, true));

            Contacts.Body after = only(resolved);
            assertTrue(resolved.hits().get(0).gripped());
            assertVec("on " + normal, along.times(100 * 3.0 / 7), across(after.velocity(), normal), 1e-9);
            assertEquals("on " + normal, 300, after.velocity().dot(normal), 1e-9);
            assertVec(
                    "on " + normal,
                    normal.cross(along).times(100 * 10.0 / 7),
                    after.spin().times(solid.radius()),
                    1e-9);
        }
    }

    @Test
    public void crossesSpinningBallDroppedStraightDownReversesItsSpinAndLeavesTheWayItsTopWasGoing() {
        Pollen solid = pollen(0.4);
        double r = solid.radius();
        Vec3 spin = new Vec3(0, 60, 0);
        Vec3 topGoes = spin.cross(UP.times(r));
        Contacts.Body dropped = body("pollen", solid, new Vec3(0, 0, -200), spin);

        Contacts.Body gripping = only(
                pressed(everywhere(bounce(0.55, 1.5, 1), 0, 0, 0), new Contacts.Pressing(dropped, FLOOR, UP, true)));
        Contacts.Body rolling = only(
                pressed(everywhere(bounce(0.55, 1.5, 0), 0, 0, 0), new Contacts.Pressing(dropped, FLOOR, UP, true)));

        assertVec("gripping", topGoes.times(4.0 / 7), across(gripping.velocity(), UP), 1e-9);
        assertVec("gripping", spin.times(-3.0 / 7), gripping.spin(), 1e-9);
        assertEquals(0.55 * 200, gripping.velocity().dot(UP), 1e-9);
        assertVec("rolling", topGoes.times(2.0 / 7), across(rolling.velocity(), UP), 1e-9);
        assertVec("rolling", spin.times(2.0 / 7), rolling.spin(), 1e-9);
        assertVec("rolling", Vec3.zero(), across(contactVelocity(rolling, UP), UP), 1e-9);
    }

    @Test
    public void aContactThatWasAlreadyTouchingDoesNotBounce() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Contacts.Body falling = body("pollen", pollen(0.634), new Vec3(30, 0, -100), Vec3.zero());

        Contacts.Resolved resolved = pressed(surfaces, new Contacts.Pressing(falling, FLOOR, UP, false));

        assertEquals(0, only(resolved).velocity().z(), 1e-12);
        assertTrue(resolved.hits().isEmpty());
    }

    @Test
    public void aSustainedContactThatCanGripStopsTheSlipAndRolls() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0);
        Contacts.Body sliding = body("pollen", pollen(0.634), new Vec3(30, 0, -100), new Vec3(0, -4, 0));

        Contacts.Body after = only(pressed(surfaces, new Contacts.Pressing(sliding, FLOOR, UP, false)));

        assertVec("slip", Vec3.zero(), across(contactVelocity(after, UP), UP), 1e-9);
        assertEquals(0, after.velocity().z(), 1e-12);
    }

    @Test
    public void aSustainedContactThatCannotGripSlowsTheSlipByTheFrictionLimit() {
        Pollen shape = pollen(0.634);
        double alpha = shape.inertiaFactor();
        Surfaces surfaces = everywhere(bounce(0.55, 0.2, 0.2), 0.1, 0.06, 0);
        Contacts.Body sliding = body("pollen", shape, new Vec3(300, 0, -10), Vec3.zero());

        Contacts.Body after = only(pressed(surfaces, new Contacts.Pressing(sliding, FLOOR, UP, false)));

        double pushed = 10 * shape.kilograms();
        double rubbed = 0.2 * pushed;
        assertVec("velocity", new Vec3(300 - rubbed / shape.kilograms(), 0, 0), after.velocity(), 1e-9);
        assertVec(
                "slip",
                new Vec3(300 - rubbed * (1 + alpha) / (alpha * shape.kilograms()), 0, 0),
                across(contactVelocity(after, UP), UP),
                1e-9);
    }

    @Test
    public void aContactThatMeetsMoreSlowlyThanTheThresholdDoesNotBounce() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Pollen shape = pollen(0.634);

        for (double approach : new double[] {0.5, 3, 4}) {
            Contacts.Resolved resolved = pressed(
                    surfaces,
                    new Contacts.Pressing(
                            body("pollen", shape, new Vec3(30, 0, -approach), Vec3.zero()), FLOOR, UP, true));

            assertEquals("at " + approach, 0, only(resolved).velocity().z(), 1e-12);
            assertTrue("at " + approach, resolved.hits().isEmpty());
        }
        Contacts.Resolved bounced = pressed(
                surfaces,
                new Contacts.Pressing(body("pollen", shape, new Vec3(30, 0, -5), Vec3.zero()), FLOOR, UP, true));
        assertEquals(0.55 * 5, only(bounced).velocity().z(), 1e-12);
        assertEquals(1, bounced.hits().size());
    }

    @Test
    public void aBallLeavingTheSurfaceIsNotHeldToIt() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Contacts.Body leaving = body("pollen", pollen(0.634), new Vec3(30, 0, 40), new Vec3(0, 3, 9));

        Contacts.Resolved resolved = pressed(surfaces, new Contacts.Pressing(leaving, FLOOR, UP, true));

        assertEquals(leaving, only(resolved));
        assertTrue(resolved.hits().isEmpty());
    }

    @Test
    public void anImpactTwistsTheSpinAboutTheNormalTowardNothingByTheLeverTimesTheImpulse() {
        Pollen shape = pollen(0.634);
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0);
        Contacts.Body spinning = body("pollen", shape, new Vec3(0, 0, -200), new Vec3(0, 0, 50));

        Contacts.Resolved bounced = pressed(surfaces, new Contacts.Pressing(spinning, FLOOR, UP, true));
        Contacts.Resolved pressing = pressed(surfaces, new Contacts.Pressing(spinning, FLOOR, UP, false));

        double impact = bounced.hits().get(0).normalImpulse();
        assertEquals(shape.kilograms() * 1.55 * 200, impact, 1e-12);
        assertVec(
                "bounced",
                new Vec3(0, 0, 50 - 0.1 * impact / inertia(shape)),
                only(bounced).spin(),
                1e-9);
        assertVec(
                "pressing",
                new Vec3(0, 0, 50 - 0.06 * shape.kilograms() * 200 / inertia(shape)),
                only(pressing).spin(),
                1e-9);
    }

    @Test
    public void aTwistNeverReversesTheSpinAboutTheNormal() {
        Pollen shape = pollen(0.634);
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.3, 0.2, 0);
        Vec3 normal = new Vec3(0.2, 0.1, 1).unit();
        for (double about : new double[] {-50, -1, 1, 50}) {
            for (double approach = 5; approach < 3000; approach += 15) {
                for (boolean fresh : new boolean[] {true, false}) {
                    Contacts.Body spinning = body("pollen", shape, normal.times(-approach), normal.times(about));

                    double after = only(pressed(surfaces, new Contacts.Pressing(spinning, FLOOR, normal, fresh)))
                            .spin()
                            .dot(normal);

                    String at = about + " rad/s at " + approach + " in/s, fresh " + fresh + ": " + after;
                    assertTrue(at, after * about >= -1e-12 * about * about);
                    assertTrue(at, Math.abs(after) <= Math.abs(about) + 1e-12);
                }
            }
        }
        Contacts.Body hard = body("pollen", shape, new Vec3(0, 0, -2000), new Vec3(0, 0, 50));
        assertEquals(
                0,
                only(pressed(surfaces, new Contacts.Pressing(hard, FLOOR, UP, true)))
                        .spin()
                        .z(),
                1e-12);
    }

    @Test
    public void aBallPressedOnASurfaceRollsLessByTheRollingResistance() {
        Pollen shape = pollen(0.634);
        double r = shape.radius(), alpha = shape.inertiaFactor();
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0, 0, 0.04);
        Contacts.Body rolling = body("pollen", shape, new Vec3(100, 0, -2), new Vec3(0, 100 / r, 0));

        Contacts.Body after = only(pressed(surfaces, new Contacts.Pressing(rolling, FLOOR, UP, false)));

        double pushed = 2 * shape.kilograms();
        assertVec("velocity", new Vec3(100, 0, 0), after.velocity(), 1e-9);
        assertVec(
                "spin",
                new Vec3(0, 100 / r - 0.04 * (1 + alpha) * pushed / (alpha * shape.kilograms() * r), 0),
                after.spin(),
                1e-9);
    }

    @Test
    public void rollingResistanceNeverTurnsTheSpinAcrossTheNormalAround() {
        Pollen shape = pollen(0.634);
        double r = shape.radius();
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0, 0, 0.2);
        Vec3 spin = new Vec3(0.3, 1, 0).times(1 / r);
        Contacts.Body slow = body("pollen", shape, spin.cross(UP.times(r)).plus(new Vec3(0, 0, -2000)), spin);

        Contacts.Body after = only(pressed(surfaces, new Contacts.Pressing(slow, FLOOR, UP, false)));

        Vec3 left = across(after.spin(), UP);
        assertTrue(left + " against " + spin, left.dot(across(spin, UP)) >= 0);
        assertVec("spin", Vec3.zero(), left, 1e-12);
    }

    @Test
    public void rollingResistanceActsOnlyOnASustainedContactWithSomethingFixed() {
        Pollen shape = pollen(0.634);
        double r = shape.radius();
        Surfaces resisting = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.2);
        Surfaces free = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0);
        Contacts.Body landing = body("pollen", shape, new Vec3(100, 0, -200), new Vec3(0, 100 / r, 3));
        Contacts.Body other = body("piece", shape, new Vec3(0, 0, 0), new Vec3(0, -40 / r, 1));
        Contacts.Body pressing = body("pollen", shape, new Vec3(100, 0, -2), new Vec3(0, 100 / r, 3));

        List<List<Contacts.Pressing>> notRolling = List.of(
                List.of(new Contacts.Pressing(landing, FLOOR, UP, true)),
                List.of(new Contacts.Pressing(landing, new Contacts.Against.Ball(other), UP, true)),
                List.of(new Contacts.Pressing(pressing, new Contacts.Against.Ball(other), UP, false)));
        for (List<Contacts.Pressing> pressings : notRolling) {
            assertEquals(
                    pressings.toString(),
                    Contacts.resolve(free, pressings).bodies(),
                    Contacts.resolve(resisting, pressings).bodies());
        }
    }

    @Test
    public void equalBallsMeetingHeadOnSwapTheirVelocitiesWeightedByTheirRestitution() {
        Pollen shape = pollen(0.634);
        Surfaces surfaces = everywhere(bounce(0.55, 0.3, 0.15), 0.1, 0.06, 0.04);
        Contacts.Body a = body("a", shape, new Vec3(100, 0, 0), Vec3.zero());
        Contacts.Body b = body("b", shape, new Vec3(-50, 0, 0), Vec3.zero());

        Contacts.Resolved resolved =
                pressed(surfaces, new Contacts.Pressing(a, new Contacts.Against.Ball(b), new Vec3(-1, 0, 0), true));

        double e = 0.55;
        assertVec("a", new Vec3(25 - e * 75, 0, 0), named("a", resolved).velocity(), 1e-9);
        assertVec("b", new Vec3(25 + e * 75, 0, 0), named("b", resolved).velocity(), 1e-9);
        assertEquals(1, resolved.hits().size());
        Contacts.Hit hit = resolved.hits().get(0);
        assertEquals("a", hit.ball());
        assertEquals("b", hit.against());
        assertEquals(Surface.PIECE, hit.surface());
        assertEquals(150, hit.approach(), 1e-12);
    }

    @Test
    public void twoBallsKeepTheirMomentumAndAngularMomentumAndNeverGainEnergy() {
        Random random = new Random(7175);
        for (int i = 0; i < 4000; i++) {
            Pollen one = randomPollen(random), two = randomPollen(random);
            Bounce bounce = randomBounce(random);
            Surfaces surfaces = Valid.value(Surfaces.of(
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    bounce,
                    0.2 * random.nextDouble(),
                    0.5 + 24.5 * random.nextDouble(),
                    0.3 * random.nextDouble(),
                    0.2 * random.nextDouble(),
                    0.2 * random.nextDouble()));
            Vec3 normal = randomUnit(random);
            Contacts.Body b = body("b", two, randomVec(random, 300), randomVec(random, 300));
            Vec3 closing = into(randomVec(random, 300), normal, 0.1 + 400 * random.nextDouble());
            Contacts.Body a = body("a", one, closing.plus(b.velocity()), randomVec(random, 300));
            boolean fresh = random.nextBoolean();
            String meeting = "meeting " + i + " of " + a + " and " + b + " on " + normal + ", fresh " + fresh;

            Contacts.Resolved resolved =
                    pressed(surfaces, new Contacts.Pressing(a, new Contacts.Against.Ball(b), normal, fresh));

            assertEquals(meeting, 2, resolved.bodies().size());
            Contacts.Body a2 = named("a", resolved), b2 = named("b", resolved);
            Vec3 momentum = momentum(a).plus(momentum(b));
            double scale = one.kilograms() * a.velocity().length()
                    + two.kilograms() * b.velocity().length();
            assertVec(meeting + ": momentum", momentum, momentum(a2).plus(momentum(b2)), 1e-12 * scale);
            assertTrue(meeting + ": energy", energy(a2) + energy(b2) <= (energy(a) + energy(b)) * (1 + 1e-12));
            Vec3 toA = normal.times(one.radius()), toB = normal.times(-two.radius());
            Vec3 angular = angularMomentumAbout(toA, a).plus(angularMomentumAbout(toB, b));
            double angularScale = inertia(one) * a.spin().length()
                    + inertia(two) * b.spin().length()
                    + one.radius() * scale
                    + two.radius() * scale;
            assertVec(
                    meeting + ": angular momentum about the contact",
                    angular,
                    angularMomentumAbout(toA, a2).plus(angularMomentumAbout(toB, b2)),
                    1e-12 * angularScale);
            double twisting = a.spin().minus(b.spin()).dot(normal);
            double twisted = a2.spin().minus(b2.spin()).dot(normal);
            assertTrue(meeting + ": twisted to " + twisted, twisted * twisting >= -1e-12 * twisting * twisting);
            assertTrue(meeting + ": twisted to " + twisted, Math.abs(twisted) <= Math.abs(twisting) * (1 + 1e-12));
        }
    }

    @Test
    public void aBallPressedIntoACornerEndsGoingIntoNeitherSide() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Pollen shape = pollen(0.634);
        List<List<Vec3>> corners = List.of(
                List.of(UP, OFF_THE_WALL),
                List.of(new Vec3(1, 0, 1).unit(), new Vec3(-1, 0, 1).unit()),
                List.of(UP, new Vec3(0.5, 0, Math.sqrt(3) / 2)));
        List<Vec3> velocities = List.of(new Vec3(-60, 20, -90), new Vec3(10, 30, -120), new Vec3(-80, 5, -30));
        for (List<Vec3> corner : corners) {
            for (Vec3 velocity : velocities) {
                for (boolean fresh : new boolean[] {true, false}) {
                    Contacts.Body ball = body("pollen", shape, velocity, new Vec3(10, -30, 15));
                    List<Contacts.Pressing> pressings = new ArrayList<>();
                    for (Vec3 normal : corner) {
                        pressings.add(new Contacts.Pressing(ball, FLOOR, normal, fresh));
                    }

                    Contacts.Body after = only(Contacts.resolve(surfaces, pressings));

                    for (Vec3 normal : corner) {
                        double going = after.velocity().dot(normal);
                        assertTrue(
                                velocity + " on " + corner + ", fresh " + fresh + ": " + going + " into " + normal,
                                going >= -1e-9);
                    }
                }
            }
        }
    }

    @Test
    public void everyBodyThatTookPartComesBackOnceInTheOrderItWasFirstSeen() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Pollen shape = pollen(0.634);
        Contacts.Body a = body("a", shape, new Vec3(0, 0, -50), Vec3.zero());
        Contacts.Body b = body("b", shape, new Vec3(0, 0, -20), Vec3.zero());
        Contacts.Body c = body("c", shape, new Vec3(0, 0, 10), Vec3.zero());

        Contacts.Resolved resolved = pressed(
                surfaces,
                new Contacts.Pressing(a, FLOOR, UP, false),
                new Contacts.Pressing(b, new Contacts.Against.Ball(a), UP, false),
                new Contacts.Pressing(c, new Contacts.Against.Ball(b), UP, false),
                new Contacts.Pressing(a, WALL, OFF_THE_WALL, false));

        List<String> names = new ArrayList<>();
        for (Contacts.Body body : resolved.bodies()) {
            names.add(body.name());
        }
        assertEquals(List.of("a", "b", "c"), names);
    }

    @Test
    public void eachContactThatBouncedIsAHitInTheOrderOfTheContacts() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Pollen shape = pollen(0.634);
        Contacts.Body a = body("a", shape, new Vec3(0, 0, -50), Vec3.zero());
        Contacts.Body b = body("b", shape, new Vec3(-30, 0, 0), Vec3.zero());
        Contacts.Body c = body("c", shape, new Vec3(-30, 0, -40), Vec3.zero());

        Contacts.Resolved resolved = pressed(
                surfaces,
                new Contacts.Pressing(c, FLOOR, UP, false),
                new Contacts.Pressing(a, FLOOR, UP, true),
                new Contacts.Pressing(b, WALL, OFF_THE_WALL, true));

        assertEquals(2, resolved.hits().size());
        assertEquals("a", resolved.hits().get(0).ball());
        assertEquals("the floor", resolved.hits().get(0).against());
        assertEquals("b", resolved.hits().get(1).ball());
        assertEquals(Surface.WALL, resolved.hits().get(1).surface());
    }

    @Test
    public void aContactWithNoDirectionOrWithItselfPushesNothing() {
        Surfaces surfaces = everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04);
        Contacts.Body ball = body("pollen", pollen(0.634), new Vec3(10, 0, -50), new Vec3(1, 2, 3));

        Contacts.Resolved nowhere = pressed(surfaces, new Contacts.Pressing(ball, FLOOR, Vec3.zero(), true));
        Contacts.Resolved itself =
                pressed(surfaces, new Contacts.Pressing(ball, new Contacts.Against.Ball(ball), UP, true));

        assertEquals(List.of(ball), nowhere.bodies());
        assertTrue(nowhere.hits().isEmpty());
        assertEquals(List.of(ball), itself.bodies());
        assertTrue(itself.hits().isEmpty());
    }

    @Test
    public void nothingPressingIsNothingResolved() {
        Contacts.Resolved resolved = Contacts.resolve(everywhere(bounce(0.55, 0.5, 0.2), 0.1, 0.06, 0.04), List.of());

        assertTrue(resolved.bodies().isEmpty());
        assertTrue(resolved.hits().isEmpty());
    }

    @Test
    public void theSolverRunsTwentyIterations() {
        assertEquals(20, Contacts.ITERATIONS);
    }
}
