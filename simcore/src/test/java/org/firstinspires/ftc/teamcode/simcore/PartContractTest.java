package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class PartContractTest {
    private static final double R = 1.4;
    private static final Collider TRAY = Carried.box("tray", new Vec3(-3, -3, 10), new Vec3(3, 3, 10.5));
    private static final Vec3 PIVOT = new Vec3(0, 0, 10.5);
    private static final Vec3 TIPPING = new Vec3(0, 3, 0);
    private static final List<Double> TIMES = List.of(0.0, 0.05, 0.1, 0.17, 0.33);

    private enum Lie {
        NONE,
        FLIPPED_SPIN,
        SPUN_ABOUT_ANOTHER_POINT,
        NO_SPIN,
        RENAMED_LATER,
        NAMED_TWICE,
        SLOWER_SWEEP,
        SAYS_STILL,
        REACHES_NOWHERE,
        BEYOND_WITHIN,
        MISNAMED,
        TIME_FROM_WHERE_IT_WAS,
        FORGETFUL,
        GAP_TWICE_AS_FAR
    }

    private static final class Lying implements Surroundings.Part {
        private final Surroundings.Part inner;
        private final Lie lie;
        private final double seconds;
        private final List<Integer> asked;

        private Lying(Surroundings.Part inner, Lie lie, double seconds, List<Integer> asked) {
            this.inner = inner;
            this.lie = lie;
            this.seconds = seconds;
            this.asked = asked;
        }

        static Lying about(Lie lie) {
            return new Lying(Carried.turning(TRAY, PIVOT, TIPPING, 0.1), lie, 0, new ArrayList<>());
        }

        @Override
        public List<String> ids() {
            if (lie == Lie.RENAMED_LATER && seconds > 0.1) {
                return List.of("renamed");
            }
            return lie == Lie.NAMED_TWICE ? List.of("tray", "tray") : inner.ids();
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return lie != Lie.REACHES_NOWHERE && inner.reaches(centre, by);
        }

        @Override
        public double sweep() {
            return lie == Lie.SLOWER_SWEEP ? inner.sweep() / 2 : inner.sweep();
        }

        @Override
        public boolean moving() {
            return lie != Lie.SAYS_STILL && inner.moving();
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            List<Surroundings.Touch> told = new ArrayList<>();
            for (Surroundings.Touch touch :
                    inner.near(centre, radius, lie == Lie.BEYOND_WITHIN ? within + 5 : within)) {
                Vec3 contact = centre.along(touch.normal(), -(radius.inches() + touch.gap()));
                Vec3 spin = touch.spin(), velocity = touch.velocity();
                String id = lie == Lie.MISNAMED ? "elsewhere" : touch.id();
                double gap = lie == Lie.GAP_TWICE_AS_FAR ? 2 * touch.gap() : touch.gap();
                if (lie == Lie.FLIPPED_SPIN) {
                    spin = spin.negated();
                    velocity = spin.cross(contact.minus(PIVOT));
                }
                if (lie == Lie.SPUN_ABOUT_ANOTHER_POINT) {
                    velocity = spin.cross(contact.minus(PIVOT.plus(new Vec3(0, 0, 3))));
                }
                if (lie == Lie.NO_SPIN) {
                    spin = Vec3.zero();
                }
                if (lie == Lie.FORGETFUL) {
                    asked.add(asked.size());
                    gap = gap - asked.size() * 1e-9;
                }
                told.add(Valid.value(Surroundings.Touch.of(id, touch.surface(), touch.normal(), gap, velocity, spin)));
            }
            return told;
        }

        @Override
        public Surroundings.Part at(double then) {
            double when = lie == Lie.TIME_FROM_WHERE_IT_WAS ? seconds + then : then;
            return new Lying(inner.at(when), lie, when, asked);
        }
    }

    private static List<PartContract.Probe> probes() {
        List<PartContract.Probe> probes = new ArrayList<>();
        for (double x : new double[] {-5, -2.9, -1, 0.3, 2.2, 4.5}) {
            for (double y : new double[] {-4, -0.5, 1.7, 3.6}) {
                for (double z : new double[] {7, 9.9, 10.3, 11.9, 13}) {
                    for (double within : new double[] {0, Shot.TOUCHING_IN, 0.5, 5}) {
                        probes.add(PartContract.Probe.of(new Vec3(x, y, z), R, within));
                    }
                }
            }
        }
        return probes;
    }

    private static List<String> breaches(Surroundings.Part part) {
        return PartContract.breaches(part, probes(), TIMES, PartContract.NO_FACE_STILL);
    }

    private static void assertBreached(String rule, Lie lie) {
        List<String> breaches = breaches(Lying.about(lie));
        boolean named = false;
        for (String breach : breaches) {
            named = named || breach.contains(rule);
        }
        assertTrue(lie + " breaches " + breaches.size() + " times, never with '" + rule + "': " + breaches, named);
    }

    @Test
    public void aCarriedPartSlidingTurningOrStartingAndStoppingKeepsThePartContract() {
        PartContract.assertKept(
                "sliding", Carried.sliding(TRAY, new Vec3(4, -3, 7), 0.1), probes(), TIMES, PartContract.NO_FACE_STILL);
        PartContract.assertKept(
                "turning", Carried.turning(TRAY, PIVOT, TIPPING, 0.1), probes(), TIMES, PartContract.NO_FACE_STILL);
        PartContract.assertKept(
                "turning about a slanted axis away from it",
                Carried.turning(TRAY, new Vec3(1, 2, 9), new Vec3(1, -2, 0.5), 0),
                probes(),
                TIMES,
                PartContract.NO_FACE_STILL);
        PartContract.assertKept(
                "sliding for a while",
                Carried.slidingUntil(TRAY, new Vec3(0, 0, 6), 0.05, 0.2),
                probes(),
                TIMES,
                PartContract.NO_FACE_STILL);
        assertEquals(List.of(), breaches(Lying.about(Lie.NONE)));
    }

    @Test
    public void partsJoinedIntoOneKeepThePartContractWithTheFacesOfTheStillOneStill() {
        Collider blue = Carried.box("blue", new Vec3(-33, -3, 10), new Vec3(-27, 3, 10.5));
        Collider red = Carried.box("red", new Vec3(27, -3, 10), new Vec3(33, 3, 10.5));
        Surroundings.Part hives = Joined.of(
                Carried.turning(blue, new Vec3(-30, 0, 10.5), new Vec3(0, 1.645, 0), 0.1),
                Carried.sliding(red, Vec3.zero(), 0));
        List<PartContract.Probe> probes = new ArrayList<>();
        for (PartContract.Probe probe : probes()) {
            probes.add(
                    new PartContract.Probe(probe.centre().plus(new Vec3(-30, 0, 0)), probe.radius(), probe.within()));
            probes.add(new PartContract.Probe(probe.centre().plus(new Vec3(30, 0, 0)), probe.radius(), probe.within()));
        }

        PartContract.assertKept(
                "hives", hives, probes, TIMES, (seconds, touch) -> touch.id().equals("red"));
        List<String> breaches = PartContract.breaches(hives, probes, TIMES, (seconds, touch) -> true);
        assertTrue(breaches.toString(), breaches.get(0).contains("moves a face that is still"));
    }

    @Test
    public void aPartsNamesAreGivenOnceEachAndStayTheSameWhereverItIsPlaced() {
        assertBreached("gives a name twice", Lie.NAMED_TWICE);
        assertBreached("names change with time", Lie.RENAMED_LATER);
    }

    @Test
    public void aPartTouchesOnlyUnderItsNamesWithinWhatIsAskedAndWhereItSaysItReaches() {
        assertBreached("touches under a name it does not have", Lie.MISNAMED);
        assertBreached("touches beyond the within asked for", Lie.BEYOND_WITHIN);
        assertBreached("touches beyond where it says it reaches", Lie.REACHES_NOWHERE);
    }

    @Test
    public void aPartIsAValueThatAnswersTheSameWhereverItWasPlacedFrom() {
        assertBreached("answers differently when asked again", Lie.FORGETFUL);
        assertBreached("answers differently placed at", Lie.TIME_FROM_WHERE_IT_WAS);
    }

    @Test
    public void aPartsFacesMoveAsItsVelocitySpinSweepAndGapSay() {
        assertBreached("gap changes at", Lie.FLIPPED_SPIN);
        assertBreached("normal turns at", Lie.FLIPPED_SPIN);
        assertBreached("gap changes at", Lie.SPUN_ABOUT_ANOTHER_POINT);
        assertBreached("normal turns at", Lie.NO_SPIN);
        assertBreached("faster than its sweep", Lie.SLOWER_SWEEP);
        assertBreached("while it says it is not moving", Lie.SAYS_STILL);
        assertBreached("not the way its gap grows", Lie.GAP_TWICE_AS_FAR);
    }
}
