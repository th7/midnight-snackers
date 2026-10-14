package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

final class PartContract {
    static final double DT = 1e-6;
    static final double NUDGE_IN = 1e-6;
    static final double SIDEWAYS_IN = 1e-4;
    static final double RATE_WITHIN = 1e-4;
    static final double FLAT_WITHIN = 1e-9;

    record Probe(Vec3 centre, Length radius, double within) {
        static Probe of(Vec3 centre, double radius, double within) {
            return new Probe(centre, Valid.length(radius), within);
        }
    }

    interface Still {
        boolean face(double seconds, Surroundings.Touch touch);
    }

    static final Still NO_FACE_STILL = (seconds, touch) -> false;

    private PartContract() {}

    static void assertKept(String what, Surroundings.Part part, List<Probe> probes, List<Double> times, Still still) {
        assertEquals(what, List.of(), breaches(part, probes, times, still));
    }

    static List<String> breaches(Surroundings.Part part, List<Probe> probes, List<Double> times, Still still) {
        List<String> breaches = new ArrayList<>();
        List<String> ids = part.ids();
        Set<String> named = new HashSet<>(ids);
        if (ids.isEmpty()) {
            breaches.add("the part has no names");
        }
        if (named.size() != ids.size()) {
            breaches.add("the part gives a name twice: " + ids);
        }
        for (String id : ids) {
            if (id.isBlank()) {
                breaches.add("the part gives a blank name: " + ids);
            }
        }
        double before = Double.NaN;
        Set<String> touchedUnder = new HashSet<>();
        Set<String> squeezedUnder = new HashSet<>();
        for (double t : times) {
            Surroundings.Part then = part.at(Valid.time(t));
            if (!then.ids().equals(ids)) {
                breaches.add("the part's names change with time: " + ids + " at first and " + then.ids() + " at " + t);
            }
            for (Probe probe : probes) {
                String where = " at " + t + " s for a ball of radius "
                        + probe.radius().inches() + " at " + probe.centre() + " within " + probe.within();
                List<Surroundings.Touch> touches = near(then, probe);
                if (!touches.equals(near(then, probe))) {
                    breaches.add("the part answers differently when asked again" + where);
                }
                if (!Double.isNaN(before)
                        && !touches.equals(near(part.at(Valid.time(before)).at(Valid.time(t)), probe))) {
                    breaches.add("the part answers differently placed at " + before + " s and then" + where);
                }
                if (!touches.isEmpty()
                        && !then.reaches(probe.centre(), probe.radius().inches() + probe.within())) {
                    breaches.add("the part touches beyond where it says it reaches" + where);
                }
                for (Surroundings.Touch touch : touches) {
                    touchedUnder.add(touch.id());
                    breaches.addAll(touchBreaches(part, then, t, probe, touch, named, still, where));
                }
                List<Surroundings.Overlap> squeezes = then.squeezing(probe.centre(), probe.radius());
                if (!squeezes.equals(then.squeezing(probe.centre(), probe.radius()))) {
                    breaches.add("the part squeezes differently when asked again" + where);
                }
                if (!Double.isNaN(before)
                        && !squeezes.equals(part.at(Valid.time(before))
                                .at(Valid.time(t))
                                .squeezing(probe.centre(), probe.radius()))) {
                    breaches.add("the part squeezes differently placed at " + before + " s and then" + where);
                }
                if (!squeezes.isEmpty() && !then.squeezes()) {
                    breaches.add("the part squeezes while it says it does not" + where);
                }
                if (!squeezes.isEmpty()
                        && !then.reaches(probe.centre(), probe.radius().inches())) {
                    breaches.add("the part squeezes beyond where it says it reaches" + where);
                }
                for (Surroundings.Overlap overlap : squeezes) {
                    squeezedUnder.add(overlap.id());
                    breaches.addAll(squeezeBreaches(part, then, t, probe, overlap, named, where));
                }
            }
            before = t;
        }
        for (String id : touchedUnder) {
            if (squeezedUnder.contains(id)) {
                breaches.add("the part touches under a name it squeezes under: " + id);
            }
        }
        return breaches;
    }

    private static List<String> squeezeBreaches(
            Surroundings.Part part,
            Surroundings.Part then,
            double t,
            Probe probe,
            Surroundings.Overlap overlap,
            Set<String> named,
            String where) {
        List<String> breaches = new ArrayList<>();
        String said = " (" + overlap + ")" + where;
        if (!named.contains(overlap.id())) {
            breaches.add("the part squeezes under a name it does not have" + said);
        }
        Optional<Surroundings.Squeeze> met = Surroundings.Squeeze.of(overlap, probe.centre(), probe.radius())
                .fold(squeeze -> Optional.of(squeeze), rule -> Optional.empty());
        if (met.isEmpty()) {
            breaches.add("the part squeezes with what moves at no finite velocity where the ball meets it" + said);
            return breaches;
        }
        Surroundings.Squeeze squeeze = met.get();
        if (!then.moving() && squeeze.moves()) {
            breaches.add("the part moves a squeezing face while it says it is not moving" + said);
        }
        Vec3 n = overlap.normal();
        double sweep = sweepAround(part, t);
        double along = squeeze.velocity().dot(n);
        if (StrictMath.abs(along) > sweep * (1 + RATE_WITHIN) + RATE_WITHIN) {
            breaches.add("the part moves a squeezing face along its normal at " + along
                    + " in/s, faster than its sweep of " + sweep + said);
        }
        Optional<Surroundings.Overlap> nudged =
                sameOverlap(then.squeezing(probe.centre().along(n, NUDGE_IN), probe.radius()), overlap);
        if (nudged.isPresent() && closeTo(nudged.get().normal(), n, FLAT_WITHIN)) {
            double shrank = overlap.depth() - nudged.get().depth();
            if (StrictMath.abs(shrank - NUDGE_IN) > NUDGE_IN * RATE_WITHIN) {
                breaches.add("the part's normal is not the way its depth shrinks: moving the ball " + NUDGE_IN
                        + " in along it shrinks the depth by " + shrank + said);
            }
        }
        Optional<Surroundings.Overlap> larger = sameOverlap(
                then.squeezing(probe.centre(), Valid.length(probe.radius().inches() + NUDGE_IN)), overlap);
        double grew = larger.map(deeper -> deeper.depth() - overlap.depth()).orElse(Double.NaN);
        if (!(StrictMath.abs(grew - NUDGE_IN) <= NUDGE_IN * RATE_WITHIN)) {
            breaches.add("the part's depth is not the ball's overlap for the radius it was asked about: a ball "
                    + NUDGE_IN + " in larger is squeezed " + grew + " in deeper" + said);
        }
        Optional<Surroundings.Overlap> earlier =
                sameOverlap(part.at(Valid.time(t - DT)).squeezing(probe.centre(), probe.radius()), overlap);
        Optional<Surroundings.Overlap> later =
                sameOverlap(part.at(Valid.time(t + DT)).squeezing(probe.centre(), probe.radius()), overlap);
        if (earlier.isPresent() && later.isPresent()) {
            double behind = (overlap.depth() - earlier.get().depth()) / DT,
                    ahead = (later.get().depth() - overlap.depth()) / DT;
            double tolerance = RATE_WITHIN * (1 + sweep);
            double rate = (later.get().depth() - earlier.get().depth()) / (2 * DT);
            if (StrictMath.abs(ahead - behind) <= tolerance && StrictMath.abs(rate - along) > tolerance) {
                breaches.add("the part's depth changes at " + rate
                        + " in/s where the face's velocity along its normal says " + along + said);
            }
        }
        return breaches;
    }

    private static Optional<Surroundings.Overlap> sameOverlap(
            List<Surroundings.Overlap> overlaps, Surroundings.Overlap overlap) {
        for (Surroundings.Overlap other : overlaps) {
            if (other.id().equals(overlap.id())) {
                return Optional.of(other);
            }
        }
        return Optional.empty();
    }

    private static List<String> touchBreaches(
            Surroundings.Part part,
            Surroundings.Part then,
            double t,
            Probe probe,
            Surroundings.Touch touch,
            Set<String> named,
            Still still,
            String where) {
        List<String> breaches = new ArrayList<>();
        String said = " (" + touch + ")" + where;
        if (!named.contains(touch.id())) {
            breaches.add("the part touches under a name it does not have" + said);
        }
        if (!(touch.gap() <= probe.within())) {
            breaches.add("the part touches beyond the within asked for" + said);
        }
        Vec3 n = touch.normal();
        if (!(Double.isFinite(touch.gap())
                && touch.velocity().isFinite()
                && touch.spin().isFinite())) {
            breaches.add("the part touches with a value that is not finite" + said);
        }
        if (!(StrictMath.abs(n.dot(n) - 1) <= Surroundings.Touch.UNIT_WITHIN)) {
            breaches.add("the part touches with a normal that is not a unit vector" + said);
        }
        if (!then.moving() && touch.moves()) {
            breaches.add("the part moves a face while it says it is not moving" + said);
        }
        if (still.face(t, touch) && touch.moves()) {
            breaches.add("the part moves a face that is still" + said);
        }
        double sweep = sweepAround(part, t);
        double along = touch.velocity().dot(n);
        if (StrictMath.abs(along) > sweep * (1 + RATE_WITHIN) + RATE_WITHIN) {
            breaches.add("the part moves a face along its normal at " + along + " in/s, faster than its sweep of "
                    + sweep + said);
        }
        breaches.addAll(gradientBreaches(then, probe, touch, said));
        breaches.addAll(rateBreaches(part, t, probe, touch, sweep, said));
        return breaches;
    }

    private static List<String> gradientBreaches(
            Surroundings.Part then, Probe probe, Surroundings.Touch touch, String said) {
        List<String> breaches = new ArrayList<>();
        Vec3 n = touch.normal();
        Optional<Surroundings.Touch> nudged =
                same(then.near(probe.centre().along(n, NUDGE_IN), probe.radius(), probe.within() + 1), touch);
        if (nudged.isPresent() && closeTo(nudged.get().normal(), n, FLAT_WITHIN)) {
            double grew = nudged.get().gap() - touch.gap();
            if (StrictMath.abs(grew - NUDGE_IN) > NUDGE_IN * RATE_WITHIN) {
                breaches.add("the part's normal is not the way its gap grows: moving the ball " + NUDGE_IN
                        + " in along it grows the gap by " + grew + said);
            }
        }
        return breaches;
    }

    private static List<String> rateBreaches(
            Surroundings.Part part, double t, Probe probe, Surroundings.Touch touch, double sweep, String said) {
        List<String> breaches = new ArrayList<>();
        Optional<Surroundings.Touch> earlier = same(near(part.at(Valid.time(t - DT)), widened(probe)), touch);
        Optional<Surroundings.Touch> later = same(near(part.at(Valid.time(t + DT)), widened(probe)), touch);
        if (earlier.isEmpty() || later.isEmpty()) {
            return breaches;
        }
        double behind = (touch.gap() - earlier.get().gap()) / DT,
                ahead = (later.get().gap() - touch.gap()) / DT;
        double tolerance = RATE_WITHIN * (1 + sweep);
        if (StrictMath.abs(ahead - behind) > tolerance) {
            return breaches;
        }
        double rate = (later.get().gap() - earlier.get().gap()) / (2 * DT);
        double told = -touch.velocity().dot(touch.normal());
        if (StrictMath.abs(rate - told) > tolerance) {
            breaches.add("the part's gap changes at " + rate + " in/s where the face's velocity along its normal says "
                    + told + said);
        }
        if (StrictMath.abs(rate) > sweep + tolerance) {
            breaches.add("the part's gap changes at " + rate + " in/s, faster than its sweep of " + sweep + said);
        }
        if (flat(part.at(Valid.time(t)), probe, touch)
                && closeTo(earlier.get().normal(), touch.normal(), 1e-3)
                && closeTo(later.get().normal(), touch.normal(), 1e-3)) {
            Vec3 turning = later.get().normal().minus(earlier.get().normal()).times(1 / (2 * DT));
            Vec3 spun = touch.spin().cross(touch.normal());
            if (!closeTo(turning, spun, tolerance)) {
                breaches.add("the part's normal turns at " + turning + " where its spin says " + spun + said);
            }
        }
        return breaches;
    }

    private static boolean flat(Surroundings.Part then, Probe probe, Surroundings.Touch touch) {
        Vec3 n = touch.normal();
        Vec3 across = n.cross(StrictMath.abs(n.x()) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0));
        Vec3 side = across.times(1 / StrictMath.sqrt(across.dot(across)));
        Vec3 other = n.cross(side);
        for (Vec3 way : List.of(side, side.negated(), other, other.negated())) {
            Optional<Surroundings.Touch> beside =
                    same(then.near(probe.centre().along(way, SIDEWAYS_IN), probe.radius(), probe.within() + 1), touch);
            if (beside.isEmpty() || !closeTo(beside.get().normal(), n, FLAT_WITHIN)) {
                return false;
            }
        }
        return true;
    }

    private static double sweepAround(Surroundings.Part part, double t) {
        double sweep = 0;
        for (double when : new double[] {t - DT, t, t + DT}) {
            Surroundings.Part then = part.at(Valid.time(when));
            if (then.moving()) {
                double said = then.sweep();
                sweep = StrictMath.max(sweep, said >= 0 ? said : Double.POSITIVE_INFINITY);
            }
        }
        return sweep;
    }

    private static Probe widened(Probe probe) {
        return new Probe(probe.centre(), probe.radius(), probe.within() + 1);
    }

    private static List<Surroundings.Touch> near(Surroundings.Part part, Probe probe) {
        return part.near(probe.centre(), probe.radius(), probe.within());
    }

    private static Optional<Surroundings.Touch> same(List<Surroundings.Touch> touches, Surroundings.Touch touch) {
        for (Surroundings.Touch other : touches) {
            if (other.id().equals(touch.id())) {
                return Optional.of(other);
            }
        }
        return Optional.empty();
    }

    private static boolean closeTo(Vec3 a, Vec3 b, double within) {
        Vec3 apart = a.minus(b);
        return apart.dot(apart) <= within * within;
    }
}
