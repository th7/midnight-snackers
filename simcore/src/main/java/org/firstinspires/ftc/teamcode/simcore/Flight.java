package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class Flight<K> {
    public static final double GRAVITY_IN_PER_S2 = 386.09;
    public static final double LANDING_SPEED_IN_PER_S = 25;
    public static final double BOUNCE = 0.3;
    public static final double FRICTION = 0.4;
    public static final double REST_SPEED_IN_PER_S = 0.5;
    public static final double REST_SECONDS = 0.25;
    public static final double ROLL_SECONDS = 0.8;

    private static final double SPIN_INERTIA = 0.4;
    private static final double CONTACT_TOLERANCE_IN = 0.02;
    private static final double RADII_PER_STEP = 0.25;
    private static final int MOST_STEPS = 64;
    private static final int SOLVER_ITERATIONS = 10;
    private static final int PUSH_ITERATIONS = 3;
    private static final double NEIGHBOURHOOD_IN = 8;
    private static final double APART_IN = 1e-9;

    public sealed interface Landing<K> permits OnTheFloor, OutOfTheField {
        K ball();

        Vec3 at();
    }

    public record OnTheFloor<K>(K ball, Vec3 at, Vec2 velocity) implements Landing<K> {}

    public record OutOfTheField<K>(K ball, Vec3 at) implements Landing<K> {}

    public record Stepped<K>(Flight<K> flight, List<Landing<K>> landings) {}

    private record Ball<K>(K key, Field.Kind kind, Length radius, Vec3 at, Vec3 velocity, Vec3 spin, double slowFor) {}

    private static final class Moving<K> {
        final K key;
        final Field.Kind kind;
        final Length length;
        final double radius;
        Vec3 at;
        Vec3 velocity;
        Vec3 spin;
        double slowFor;
        boolean awake = true;
        boolean resting;

        Moving(Ball<K> ball) {
            this.key = ball.key();
            this.kind = ball.kind();
            this.length = ball.radius();
            this.radius = ball.radius().inches();
            this.at = ball.at();
            this.velocity = ball.velocity();
            this.spin = ball.spin();
            this.slowFor = ball.slowFor();
        }

        double inverseMass() {
            return awake ? 1 : 0;
        }

        void move(Vec3 direction, double distance) {
            at = at.along(direction, distance);
        }

        Ball<K> still() {
            return new Ball<>(key, kind, length, at, velocity, spin, slowFor);
        }
    }

    private abstract static class Contact<K> {
        final Moving<K> a;
        final Vec3 normal;
        double bounceTo;
        double pushed;
        Vec3 rubbed = Vec3.zero();

        Contact(Moving<K> a, Vec3 normal) {
            this.a = a;
            this.normal = normal;
        }

        abstract Vec3 theirs();

        abstract double theirInverseMass();

        abstract void shoveThem(Vec3 impulse, double ib);

        abstract void spinThem(Vec3 turn, double ib);

        abstract void rest();

        Contact<K> bouncing() {
            double closing = relative().dot(normal);
            bounceTo = -closing > LANDING_SPEED_IN_PER_S ? -BOUNCE * closing : 0;
            return this;
        }

        Vec3 relative() {
            Vec3 mine = a.velocity.along(a.spin.cross(normal), -a.radius);
            return mine.minus(theirs());
        }

        void solve() {
            double ia = a.inverseMass(), ib = theirInverseMass();
            if (ia + ib == 0) {
                return;
            }
            double push = (bounceTo - relative().dot(normal)) / (ia + ib);
            double total = Math.max(0, pushed + push);
            push = total - pushed;
            pushed = total;
            shove(normal.times(push), ia, ib);

            Vec3 slip = relative();
            slip = slip.along(normal, -slip.dot(normal));
            Vec3 rub = slip.times(-1 / ((ia + ib) * (1 + 1 / SPIN_INERTIA)));
            Vec3 rubbing = rubbed.along(rub, 1);
            double most = FRICTION * pushed;
            if (rubbing.length() > most) {
                rubbing = rubbing.times(most / rubbing.length());
            }
            rub = rubbing.minus(rubbed);
            rubbed = rubbing;
            shove(rub, ia, ib);
            a.spin = a.spin.along(normal.cross(rub), -ia / (SPIN_INERTIA * a.radius));
            spinThem(normal.cross(rub), ib);
        }

        private void shove(Vec3 impulse, double ia, double ib) {
            a.velocity = a.velocity.along(impulse, ia);
            shoveThem(impulse, ib);
        }
    }

    private static final class AgainstAHive<K> extends Contact<K> {
        final Vec3 surface;

        AgainstAHive(Moving<K> a, Vec3 normal, Vec3 surface) {
            super(a, normal);
            this.surface = surface;
        }

        @Override
        Vec3 theirs() {
            return surface;
        }

        @Override
        double theirInverseMass() {
            return 0;
        }

        @Override
        void shoveThem(Vec3 impulse, double ib) {}

        @Override
        void spinThem(Vec3 turn, double ib) {}

        @Override
        void rest() {
            a.resting = true;
        }
    }

    private static final class BetweenBalls<K> extends Contact<K> {
        final Moving<K> b;

        BetweenBalls(Moving<K> a, Moving<K> b, Vec3 normal) {
            super(a, normal);
            this.b = b;
        }

        @Override
        Vec3 theirs() {
            return b.velocity.along(b.spin.cross(normal), b.radius);
        }

        @Override
        double theirInverseMass() {
            return b.inverseMass();
        }

        @Override
        void shoveThem(Vec3 impulse, double ib) {
            b.velocity = new Vec3(
                    b.velocity.x() - impulse.x() * ib,
                    b.velocity.y() - impulse.y() * ib,
                    b.velocity.z() - impulse.z() * ib);
        }

        @Override
        void spinThem(Vec3 turn, double ib) {
            b.spin = b.spin.along(turn, -ib / (SPIN_INERTIA * b.radius));
        }

        @Override
        void rest() {
            a.resting = true;
            b.resting = true;
        }
    }

    private interface Pair<K> {
        void of(Moving<K> one, Moving<K> other);
    }

    private final Hives hives;
    private final List<Ball<K>> balls;

    private Flight(Hives hives, List<Ball<K>> balls) {
        this.hives = hives;
        this.balls = Collections.unmodifiableList(balls);
    }

    public static <K> Flight<K> over(Hives hives) {
        return new Flight<>(hives, List.of());
    }

    public Hives hives() {
        return hives;
    }

    public Flight<K> with(K ball, Field.Kind kind, Length radius, Vec3 at, Vec3 velocity) {
        List<Ball<K>> next = new ArrayList<>(without(ball).balls);
        next.add(new Ball<>(ball, kind, radius, at, velocity, Vec3.zero(), 0));
        return new Flight<>(hives, next);
    }

    public Checked<Flight<K>> within(K ball, Field.Kind kind, Length radius, Field.Cell cell) {
        Flight<K> without = without(ball);
        for (Vec3 spot : hives.restingSpots(cell, radius)) {
            if (without.clear(spot, radius.inches())) {
                return Checked.ok(without.with(ball, kind, radius, spot, Vec3.zero()));
            }
        }
        return Checked.rejected("there is no room left in " + cell.name() + " for another " + kind.named());
    }

    private boolean clear(Vec3 spot, double radius) {
        for (Ball<K> other : balls) {
            if (other.at().minus(spot).length() < other.radius().inches() + radius - CONTACT_TOLERANCE_IN) {
                return false;
            }
        }
        return true;
    }

    public Flight<K> without(K ball) {
        List<Ball<K>> next = new ArrayList<>();
        for (Ball<K> flying : balls) {
            if (!flying.key().equals(ball)) {
                next.add(flying);
            }
        }
        return new Flight<>(hives, next);
    }

    public boolean holds(K ball) {
        return at(ball).isPresent();
    }

    public Optional<Vec3> at(K ball) {
        for (Ball<K> flying : balls) {
            if (flying.key().equals(ball)) {
                return Optional.of(flying.at());
            }
        }
        return Optional.empty();
    }

    public int fill(Field.Hive hive) {
        return fill(balls, hives, hive);
    }

    public Optional<Double> load(String alliance) {
        return hives.hiveOf(alliance).map(hive -> fill(hive) / (double) Hives.FULL);
    }

    public int scored(String alliance) {
        return scored().getOrDefault(alliance, 0);
    }

    public Map<String, Integer> scored() {
        Map<String, Integer> out = new LinkedHashMap<>();
        for (Field.Cell cell : hives.field().cells()) {
            out.putIfAbsent(cell.alliance(), 0);
        }
        for (Ball<K> ball : balls) {
            hives.cellHolding(ball.at()).ifPresent(cell -> out.merge(cell.alliance(), 1, Integer::sum));
        }
        return out;
    }

    public Stepped<K> step(Seconds seconds) {
        List<Moving<K>> moving = new ArrayList<>();
        for (Ball<K> ball : balls) {
            moving.add(new Moving<>(ball));
        }
        double fastest = 0;
        for (Moving<K> ball : moving) {
            fastest = Math.max(fastest, (ball.velocity.length() + GRAVITY_IN_PER_S2 * seconds.value()) / ball.radius);
        }
        int steps = (int) Math.min(MOST_STEPS, Math.max(1, Math.ceil(seconds.value() * fastest / RADII_PER_STEP)));
        Seconds dt = seconds.dividedInto(steps);
        List<Landing<K>> landed = new ArrayList<>();
        Hives now = hives;
        for (int i = 0; i < steps; i++) {
            now = substep(moving, now, dt, landed);
        }
        List<Ball<K>> still = new ArrayList<>();
        for (Moving<K> ball : moving) {
            still.add(ball.still());
        }
        return new Stepped<>(new Flight<>(now, still), Collections.unmodifiableList(landed));
    }

    private static <K> Hives substep(List<Moving<K>> balls, Hives hives, Seconds step, List<Landing<K>> landed) {
        double dt = step.value();
        wake(balls, hives);
        for (Moving<K> ball : balls) {
            if (ball.awake) {
                ball.velocity =
                        new Vec3(ball.velocity.x(), ball.velocity.y(), ball.velocity.z() - GRAVITY_IN_PER_S2 * dt);
                ball.resting = false;
            }
        }
        List<Contact<K>> contacts = contacts(balls, hives);
        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            for (Contact<K> contact : contacts) {
                contact.solve();
            }
        }
        for (Contact<K> contact : contacts) {
            if (contact.pushed > 0) {
                contact.rest();
            }
        }
        for (Moving<K> ball : balls) {
            if (!ball.awake) {
                continue;
            }
            if (ball.resting) {
                double rolling = 1 / (1 + dt / ROLL_SECONDS);
                ball.velocity = ball.velocity.times(rolling);
                ball.spin = ball.spin.times(rolling);
            }
            ball.at = ball.at.along(ball.velocity, dt);
        }
        Hives after = hives.after(step);
        pushApart(balls, after);
        meetTheFloorAndTheWalls(balls, after.field(), landed);
        for (Moving<K> ball : balls) {
            if (ball.awake) {
                double fastest = ball.velocity.length() + ball.spin.length() * ball.radius;
                ball.slowFor = fastest <= REST_SPEED_IN_PER_S ? ball.slowFor + dt : 0;
            }
        }
        for (Field.Hive hive : after.field().hives()) {
            if (!after.tipping(hive) && fill(stillOf(balls), after, hive) >= Hives.FULL) {
                after = after.tipped(hive);
            }
        }
        return after;
    }

    private static <K> List<Ball<K>> stillOf(List<Moving<K>> balls) {
        List<Ball<K>> still = new ArrayList<>();
        for (Moving<K> ball : balls) {
            still.add(ball.still());
        }
        return still;
    }

    private static <K> int fill(List<Ball<K>> balls, Hives hives, Field.Hive hive) {
        Optional<Field.Cell> up = hives.upturnedCell(hive.alliance());
        int fill = 0;
        for (Ball<K> ball : balls) {
            if (hives.cellHolding(ball.at()).equals(up)) {
                fill += Hives.fills(ball.kind());
            }
        }
        return fill;
    }

    private static <K> void wake(List<Moving<K>> balls, Hives hives) {
        List<Field.Hive> stirring = new ArrayList<>();
        for (Field.Hive hive : hives.field().hives()) {
            boolean moving = hives.tipping(hive);
            for (Moving<K> ball : balls) {
                moving |= ball.slowFor < REST_SECONDS && hives.near(hive, ball.at, ball.radius + NEIGHBOURHOOD_IN);
            }
            if (moving) {
                stirring.add(hive);
            }
        }
        for (Moving<K> ball : balls) {
            boolean nearAny = false, nearStirring = false;
            for (Field.Hive hive : hives.field().hives()) {
                if (hives.near(hive, ball.at, ball.radius + NEIGHBOURHOOD_IN)) {
                    nearAny = true;
                    nearStirring |= stirring.contains(hive);
                }
            }
            ball.awake = !nearAny || nearStirring;
            if (!ball.awake) {
                ball.velocity = Vec3.zero();
                ball.spin = Vec3.zero();
            }
        }
    }

    private static <K> List<Contact<K>> contacts(List<Moving<K>> balls, Hives hives) {
        List<Contact<K>> contacts = new ArrayList<>();
        for (Moving<K> ball : balls) {
            if (!ball.awake) {
                continue;
            }
            for (Hives.Touch touch : hives.touching(ball.at, ball.length)) {
                contacts.add(new AgainstAHive<>(ball, touch.normal(), touch.velocity()).bouncing());
            }
        }
        pairs(balls, (a, b) -> {
            if (!a.awake && !b.awake) {
                return;
            }
            Vec3 between = a.at.minus(b.at);
            double apart = between.length();
            if (apart < a.radius + b.radius + CONTACT_TOLERANCE_IN && apart > APART_IN) {
                contacts.add(new BetweenBalls<>(a, b, between.times(1 / apart)).bouncing());
            }
        });
        return contacts;
    }

    private static <K> void pushApart(List<Moving<K>> balls, Hives hives) {
        for (int i = 0; i < PUSH_ITERATIONS; i++) {
            for (Moving<K> ball : balls) {
                if (!ball.awake) {
                    continue;
                }
                Optional<Hives.Touch> deepest = Optional.empty();
                for (Hives.Touch touch : hives.touching(ball.at, ball.length)) {
                    if (deepest.map(d -> touch.depth() > d.depth()).orElse(true)) {
                        deepest = Optional.of(touch);
                    }
                }
                deepest.ifPresent(d -> {
                    if (d.depth() > CONTACT_TOLERANCE_IN) {
                        ball.move(d.normal(), d.depth() - CONTACT_TOLERANCE_IN);
                    }
                });
            }
            pairs(balls, (one, other) -> {
                double ia = one.inverseMass(), ib = other.inverseMass();
                if (ia + ib == 0) {
                    return;
                }
                Vec3 between = one.at.minus(other.at);
                double apart = between.length();
                double over = one.radius + other.radius - apart;
                if (over > CONTACT_TOLERANCE_IN && apart > APART_IN) {
                    Vec3 normal = between.times(1 / apart);
                    one.move(normal, (over - CONTACT_TOLERANCE_IN) * ia / (ia + ib));
                    other.move(normal, -(over - CONTACT_TOLERANCE_IN) * ib / (ia + ib));
                }
            });
        }
    }

    private static <K> void pairs(List<Moving<K>> balls, Pair<K> pair) {
        for (Moving<K> one : balls) {
            boolean after = false;
            for (Moving<K> other : balls) {
                if (after) {
                    pair.of(one, other);
                }
                if (other == one) {
                    after = true;
                }
            }
        }
    }

    private static <K> void meetTheFloorAndTheWalls(List<Moving<K>> balls, Field field, List<Landing<K>> landed) {
        for (Moving<K> ball : new ArrayList<>(balls)) {
            if (!ball.awake) {
                continue;
            }
            if (ball.at.z() < ball.radius) {
                ball.at = new Vec3(ball.at.x(), ball.at.y(), ball.radius);
                if (-ball.velocity.z() < LANDING_SPEED_IN_PER_S) {
                    balls.remove(ball);
                    landed.add(new OnTheFloor<>(ball.key, ball.at, new Vec2(ball.velocity.x(), ball.velocity.y())));
                    continue;
                }
                ball.velocity = new Vec3(ball.velocity.x(), ball.velocity.y(), -ball.velocity.z() * BOUNCE);
            }
            double limit = field.size() / 2 - ball.radius;
            if (Math.abs(ball.at.x()) > limit) {
                if (outOver(ball, field, landed)) {
                    balls.remove(ball);
                    continue;
                }
                ball.at = new Vec3(Math.signum(ball.at.x()) * limit, ball.at.y(), ball.at.z());
                ball.velocity = new Vec3(-ball.velocity.x() * BOUNCE, ball.velocity.y(), ball.velocity.z());
            }
            if (Math.abs(ball.at.y()) > limit) {
                if (outOver(ball, field, landed)) {
                    balls.remove(ball);
                    continue;
                }
                ball.at = new Vec3(ball.at.x(), Math.signum(ball.at.y()) * limit, ball.at.z());
                ball.velocity = new Vec3(ball.velocity.x(), -ball.velocity.y() * BOUNCE, ball.velocity.z());
            }
        }
    }

    private static <K> boolean outOver(Moving<K> ball, Field field, List<Landing<K>> landed) {
        if (ball.at.z() - ball.radius > field.wallHeight()) {
            landed.add(new OutOfTheField<>(ball.key, new Vec3(ball.at.x(), ball.at.y(), ball.radius)));
            return true;
        }
        return false;
    }
}
