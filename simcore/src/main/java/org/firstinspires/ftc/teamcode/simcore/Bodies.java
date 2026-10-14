package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

final class Bodies {
    enum Life {
        AWAKE,
        ASLEEP,
        FIXED,
        GONE
    }

    enum Sleep {
        WHEN_RESTING,
        NEVER
    }

    static final class Step {
        static final double LONGEST_SECONDS = 0.01;

        private final Seconds seconds;

        private Step(Seconds seconds) {
            this.seconds = seconds;
        }

        static Checked<Step> of(Seconds seconds) {
            if (!(seconds.value() > 0 && seconds.value() <= LONGEST_SECONDS)) {
                return Checked.rejected(
                        "a step is more than 0 and at most " + LONGEST_SECONDS + " s, not " + seconds.value());
            }
            return Checked.ok(new Step(seconds));
        }

        Seconds seconds() {
            return seconds;
        }
    }

    sealed interface Start permits Start.Awake, Start.Asleep, Start.Fixed {
        Life life();

        record Awake(Sleep sleep) implements Start {
            @Override
            public Life life() {
                return Life.AWAKE;
            }
        }

        record Asleep() implements Start {
            @Override
            public Life life() {
                return Life.ASLEEP;
            }
        }

        record Fixed() implements Start {
            @Override
            public Life life() {
                return Life.FIXED;
            }
        }
    }

    static final class Name {
        private final String text;

        private Name(String text) {
            this.text = text;
        }

        static Checked<Name> of(String text) {
            if (text.isBlank()) {
                return Checked.rejected("a body has a name, not '" + text + "'");
            }
            return Checked.ok(new Name(text));
        }

        static Name launched() {
            return new Name(Shot.LAUNCHED);
        }

        String text() {
            return text;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Name that && text.equals(that.text);
        }

        @Override
        public int hashCode() {
            return text.hashCode();
        }

        @Override
        public String toString() {
            return text;
        }
    }

    static final class Motion {
        private final Vec3 at;
        private final Vec3 velocity;
        private final Vec3 spin;

        private Motion(Vec3 at, Vec3 velocity, Vec3 spin) {
            this.at = at;
            this.velocity = velocity;
            this.spin = spin;
        }

        static Checked<Motion> of(Vec3 at, Vec3 velocity, Vec3 spin) {
            if (!(at.isFinite() && velocity.isFinite() && spin.isFinite())) {
                return Checked.rejected("a body is at a finite place, moving and spinning at finite rates, not at " + at
                        + " moving at " + velocity + " spinning at " + spin);
            }
            return Checked.ok(new Motion(at, velocity, spin));
        }

        static Motion of(WheelPair.Launched launched) {
            return new Motion(launched.at(), launched.velocity(), launched.spin());
        }

        Vec3 at() {
            return at;
        }

        Vec3 velocity() {
            return velocity;
        }

        Vec3 spin() {
            return spin;
        }
    }

    record Entry(Name name, Ball shape, Motion motion, Start start) {}

    static final class Pieces {
        private final List<Entry> entries;

        private Pieces(List<Entry> entries) {
            this.entries = entries;
        }

        static Pieces none() {
            return new Pieces(List.of());
        }

        Checked<Pieces> with(Entry entry) {
            boolean taken = entry.name().equals(Name.launched());
            for (Entry piece : entries) {
                taken = taken || piece.name().equals(entry.name());
            }
            if (taken) {
                return Checked.rejected("every body has a name of its own, and '" + entry.name() + "' is not one");
            }
            List<Entry> more = new ArrayList<>(entries);
            more.add(entry);
            return Checked.ok(new Pieces(Collections.unmodifiableList(more)));
        }
    }

    record Seen(String name, Vec3 at, Vec3 velocity, Vec3 spin, Turn turn, Life life, double restFor) {}

    record Touching(Set<String> colliders, Set<String> bodies) {}

    record Gone(String name, Colliders.Side side, Vec3 at, Turn turn) {}

    record Span(
            double began,
            double took,
            double now,
            Vec3 from,
            List<Contacts.Pressing> pressing,
            Optional<Contacts.Resolved> resolved,
            Seen before,
            Seen atResolve,
            List<Seen> woken,
            List<Seen> slept,
            Seen after) {}

    private record Kin(Vec3 at, Vec3 velocity, Vec3 spin) {}

    private static final class Body {
        private final String name;
        private final Ball shape;
        private final Sleep sleep;
        private Vec3 at;
        private Vec3 velocity;
        private Vec3 spin;
        private Turn turn;
        private Life life;
        private double restFor;
        private Kin start;
        private Kin trial;
        private boolean touched;
        private boolean carried;
        private final Set<String> touchingColliders = new HashSet<>();
        private final Set<String> touchingBodies = new HashSet<>();

        private Body(
                String name,
                Ball shape,
                Sleep sleep,
                Vec3 at,
                Vec3 velocity,
                Vec3 spin,
                Turn turn,
                Life life,
                double restFor) {
            this.name = name;
            this.shape = shape;
            this.sleep = sleep;
            this.at = at;
            this.velocity = velocity;
            this.spin = spin;
            this.turn = turn;
            this.life = life;
            this.restFor = restFor;
            this.start = new Kin(at, velocity, spin);
            this.trial = start;
        }

        static Body of(Entry entry) {
            Motion motion = entry.motion();
            Start start = entry.start();
            return new Body(
                    entry.name().text(),
                    entry.shape(),
                    start instanceof Start.Awake awake ? awake.sleep() : Sleep.WHEN_RESTING,
                    motion.at(),
                    motion.velocity(),
                    motion.spin(),
                    Turn.identity(),
                    start.life(),
                    0);
        }

        Body copy() {
            Body copy = new Body(name, shape, sleep, at, velocity, spin, turn, life, restFor);
            copy.touchingColliders.addAll(touchingColliders);
            copy.touchingBodies.addAll(touchingBodies);
            return copy;
        }

        double radius() {
            return shape.radius();
        }

        Length reach() {
            return shape.radiusLength();
        }

        double inverseMass() {
            return life == Life.AWAKE ? 1 / shape.kilograms() : 0;
        }

        Contacts.Body contact() {
            return new Contacts.Body(name, shape, velocity, spin);
        }

        Seen seen() {
            return new Seen(name, at, velocity, spin, turn, life, restFor);
        }
    }

    private record Glimpse(Body body, Vec3 from) {
        static Glimpse of(Body body) {
            return new Glimpse(body, body.at);
        }

        Seen seen() {
            return body.seen();
        }

        Span span(
                double began,
                double took,
                double now,
                List<Contacts.Pressing> pressing,
                Optional<Contacts.Resolved> resolved,
                Seen before,
                Seen atResolve,
                List<Seen> woken,
                List<Seen> slept) {
            return new Span(
                    began,
                    took,
                    now,
                    from,
                    Collections.unmodifiableList(pressing),
                    resolved,
                    before,
                    atResolve,
                    Collections.unmodifiableList(woken),
                    Collections.unmodifiableList(slept),
                    body.seen());
        }
    }

    private final Air air;
    private final Surfaces surfaces;
    private final double step;
    private final Optional<String> watching;
    private final List<Body> bodies;
    private Surroundings surroundings;
    private long steps;
    private double now;
    private long work;

    private Bodies(
            Surroundings surroundings,
            Air air,
            Surfaces surfaces,
            double step,
            Optional<String> watching,
            List<Body> bodies,
            long steps,
            double now,
            long work) {
        this.surroundings = surroundings;
        this.air = air;
        this.surfaces = surfaces;
        this.step = step;
        this.watching = watching;
        this.bodies = bodies;
        this.steps = steps;
        this.now = now;
        this.work = work;
    }

    static Bodies watching(Surroundings surroundings, Air air, Surfaces surfaces, Step step, Name name) {
        return new Bodies(
                surroundings,
                air,
                surfaces,
                step.seconds().value(),
                Optional.of(name.text()),
                new ArrayList<>(),
                0,
                0,
                0);
    }

    static Bodies unwatched(Surroundings surroundings, Air air, Surfaces surfaces, Step step) {
        return new Bodies(
                surroundings, air, surfaces, step.seconds().value(), Optional.empty(), new ArrayList<>(), 0, 0, 0);
    }

    static Bodies launchedAmong(
            Surroundings surroundings,
            Air air,
            Surfaces surfaces,
            Step step,
            Ball pollen,
            Motion launched,
            Pieces pieces) {
        List<Body> bodies = new ArrayList<>();
        bodies.add(Body.of(new Entry(Name.launched(), pollen, launched, new Start.Awake(Sleep.NEVER))));
        for (Entry piece : pieces.entries) {
            bodies.add(Body.of(piece));
        }
        Bodies engine = new Bodies(
                surroundings, air, surfaces, step.seconds().value(), Optional.of(Shot.LAUNCHED), bodies, 0, 0, 0);
        engine.touchingAtTheStart();
        return engine;
    }

    Bodies copy() {
        List<Body> copied = new ArrayList<>();
        for (Body body : bodies) {
            copied.add(body.copy());
        }
        return new Bodies(surroundings, air, surfaces, step, watching, copied, steps, now, work);
    }

    void surroundedBy(Surroundings surroundings) {
        this.surroundings = placed(surroundings, now);
    }

    Checked<Seen> add(Name name, Ball shape, Motion motion, Start start) {
        if (named(name.text()).isPresent()) {
            return Checked.rejected("every body has a name of its own, and '" + name + "' is not one");
        }
        Body body = Body.of(new Entry(name, shape, motion, start));
        bodies.add(body);
        for (Body other : bodies) {
            if (other != body && other.life == Life.AWAKE && within(other.at, other, body.at, body, Shot.TOUCHING_IN)) {
                other.touchingBodies.add(body.name);
            }
        }
        refresh(body);
        return Checked.ok(body.seen());
    }

    Optional<Seen> remove(String name) {
        Optional<Body> named = named(name);
        named.ifPresent(removed -> {
            bodies.remove(removed);
            List<Body> woken = new ArrayList<>();
            for (Body body : bodies) {
                boolean touched = body.touchingBodies.remove(name)
                        || (removed.life != Life.GONE && within(body.at, body, removed.at, removed, Shot.TOUCHING_IN));
                if (touched && body.life != Life.GONE) {
                    body.restFor = 0;
                }
                if (touched && body.life == Life.ASLEEP) {
                    body.life = Life.AWAKE;
                    woken.add(body);
                }
            }
            for (Body body : woken) {
                refresh(body);
            }
        });
        return named.map(Body::seen);
    }

    Optional<Seen> body(String name) {
        return named(name).map(Body::seen);
    }

    Optional<Touching> touching(String name) {
        return named(name)
                .map(body -> new Touching(
                        Collections.unmodifiableSet(new HashSet<>(body.touchingColliders)),
                        Collections.unmodifiableSet(new HashSet<>(body.touchingBodies))));
    }

    List<Seen> seen() {
        List<Seen> seen = new ArrayList<>();
        for (Body body : bodies) {
            seen.add(body.seen());
        }
        return Collections.unmodifiableList(seen);
    }

    boolean anyAwakeBut(String spared) {
        for (Body body : bodies) {
            if (body.life == Life.AWAKE && !body.name.equals(spared)) {
                return true;
            }
        }
        return false;
    }

    long steps() {
        return steps;
    }

    double now() {
        return now;
    }

    long work() {
        return work;
    }

    private Optional<Body> named(String name) {
        for (Body body : bodies) {
            if (body.name.equals(name)) {
                return Optional.of(body);
            }
        }
        return Optional.empty();
    }

    List<Span> step() {
        List<Span> spans = new ArrayList<>();
        Surroundings ahead = placed(surroundings, (steps + 1) * step);
        int pieces = piecesOfThisStep(ahead);
        for (int i = 1; i <= pieces; i++) {
            aPiece((steps + (double) i / pieces) * step, spans);
        }
        steps++;
        now = steps * step;
        return Collections.unmodifiableList(spans);
    }

    List<Gone> goneOver(String spared) {
        List<Gone> gone = new ArrayList<>();
        for (Body body : bodies) {
            if (body.name.equals(spared) || body.life != Life.AWAKE) {
                continue;
            }
            surroundings.over(body.at, body.reach()).ifPresent(side -> {
                body.life = Life.GONE;
                gone.add(new Gone(body.name, side, body.at, body.turn));
            });
        }
        return Collections.unmodifiableList(gone);
    }

    private int piecesOfThisStep(Surroundings ahead) {
        double fastest = 0;
        for (Body body : bodies) {
            if (body.life == Life.AWAKE) {
                fastest = StrictMath.max(fastest, StrictMath.sqrt(body.velocity.dot(body.velocity)));
            }
        }
        double sweep = StrictMath.max(surroundings.sweep(), ahead.sweep());
        double needed = StrictMath.ceil((fastest + sweep) * step / Shot.MOST_TRAVEL_IN);
        if (Double.isNaN(needed)) {
            return Shot.MOST_PIECES_PER_STEP;
        }
        return (int) StrictMath.max(1, StrictMath.min(Shot.MOST_PIECES_PER_STEP, needed));
    }

    private List<Body> awake() {
        List<Body> awake = new ArrayList<>();
        for (Body body : bodies) {
            if (body.life == Life.AWAKE) {
                awake.add(body);
            }
        }
        return awake;
    }

    private void touchingAtTheStart() {
        for (Body body : bodies) {
            refresh(body);
        }
    }

    private void refresh(Body body) {
        body.touchingColliders.clear();
        body.touchingBodies.clear();
        if (body.life == Life.AWAKE) {
            touch(body);
        }
    }

    private void touch(Body body) {
        for (Surroundings.Touch near : near(surroundings, body.at, body.reach(), Shot.TOUCHING_IN)) {
            body.touchingColliders.add(near.id());
        }
        for (Body other : bodies) {
            if (other != body && other.life != Life.GONE && within(body.at, body, other.at, other, Shot.TOUCHING_IN)) {
                body.touchingBodies.add(other.name);
            }
        }
    }

    private static boolean within(Vec3 a, Body one, Vec3 b, Body other, double gap) {
        Vec3 apart = a.minus(b);
        double reach = one.radius() + other.radius() + gap;
        return apart.dot(apart) <= reach * reach;
    }

    private static boolean overlapping(Vec3 a, Body one, Vec3 b, Body other) {
        Vec3 apart = a.minus(b);
        double reach = one.radius() + other.radius();
        return apart.dot(apart) < reach * reach;
    }

    private Optional<Body> watched() {
        return watching.flatMap(this::named);
    }

    private void aPiece(double target, List<Span> spans) {
        int events = 0;
        while (now < target) {
            List<Body> awake = awake();
            double span = target - now;
            for (Body body : awake) {
                body.start = new Kin(body.at, body.velocity, body.spin);
            }
            Optional<Glimpse> glimpse = watched().map(Glimpse::of);
            integrate(awake, span);
            double covered = span;
            if (events < Shot.MOST_EVENTS_PER_PIECE && overlapsSomethingNew(awake, span)) {
                double lo = 0, hi = 1;
                for (int i = 0; i < Shot.BISECTIONS; i++) {
                    double mid = (lo + hi) / 2;
                    double over = mid * span;
                    integrate(awake, over);
                    if (overlapsSomethingNew(awake, over)) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                integrate(awake, lo * span);
                covered = lo * span;
                events++;
            }
            for (Body body : awake) {
                body.at = body.trial.at();
                body.velocity = body.trial.velocity();
                body.spin = body.trial.spin();
                body.turn = body.turn.turnedBy(body.start.spin(), covered);
            }
            double began = now;
            double took = covered;
            now = covered == span ? target : now + covered;
            surroundings = placed(surroundings, now);
            List<Seen> woken = wake();
            List<Contacts.Pressing> pressing = pressings();
            Optional<Seen> before = glimpse.map(Glimpse::seen);
            Optional<Contacts.Resolved> resolved = resolve(pressing);
            Optional<Seen> atResolve = glimpse.map(Glimpse::seen);
            project();
            List<Seen> slept = settle(covered);
            touchingAtTheStart();
            double ended = now;
            glimpse.flatMap(watched -> before.flatMap(pressed -> atResolve.map(
                            held -> watched.span(began, took, ended, pressing, resolved, pressed, held, woken, slept))))
                    .ifPresent(stretch -> spans.add(stretch));
        }
    }

    private void integrate(List<Body> awake, double over) {
        for (Body body : awake) {
            body.trial = rk4(body, body.start, over);
            work++;
        }
    }

    private Kin rk4(Body body, Kin from, double dt) {
        Air.Motion m1 = air.motion(body.shape, from.velocity(), from.spin());
        Kin k1 = new Kin(from.velocity(), m1.acceleration(), m1.spinRate());
        Kin s2 = along(from, k1, dt / 2);
        Air.Motion m2 = air.motion(body.shape, s2.velocity(), s2.spin());
        Kin k2 = new Kin(s2.velocity(), m2.acceleration(), m2.spinRate());
        Kin s3 = along(from, k2, dt / 2);
        Air.Motion m3 = air.motion(body.shape, s3.velocity(), s3.spin());
        Kin k3 = new Kin(s3.velocity(), m3.acceleration(), m3.spinRate());
        Kin s4 = along(from, k3, dt);
        Air.Motion m4 = air.motion(body.shape, s4.velocity(), s4.spin());
        Kin k4 = new Kin(s4.velocity(), m4.acceleration(), m4.spinRate());
        double sixth = dt / 6;
        return new Kin(
                from.at().plus(weighted(k1.at(), k2.at(), k3.at(), k4.at()).times(sixth)),
                from.velocity()
                        .plus(weighted(k1.velocity(), k2.velocity(), k3.velocity(), k4.velocity())
                                .times(sixth)),
                from.spin()
                        .plus(weighted(k1.spin(), k2.spin(), k3.spin(), k4.spin())
                                .times(sixth)));
    }

    private static Kin along(Kin from, Kin rate, double dt) {
        return new Kin(
                from.at().along(rate.at(), dt),
                from.velocity().along(rate.velocity(), dt),
                from.spin().along(rate.spin(), dt));
    }

    private static Vec3 weighted(Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
        return a.plus(b.times(2)).plus(c.times(2)).plus(d);
    }

    private boolean overlapsSomethingNew(List<Body> awake, double over) {
        if (awake.isEmpty()) {
            return false;
        }
        Surroundings then = placed(surroundings, now + over);
        for (Body body : awake) {
            for (Surroundings.Touch near : near(then, body.trial.at(), body.reach(), 0)) {
                if (near.gap() < 0 && !body.touchingColliders.contains(near.id())) {
                    return true;
                }
            }
            for (Body other : bodies) {
                if (other == body || other.life == Life.GONE || body.touchingBodies.contains(other.name)) {
                    continue;
                }
                Vec3 there = other.life == Life.AWAKE ? other.trial.at() : other.at;
                if (overlapping(body.trial.at(), body, there, other)) {
                    return true;
                }
            }
        }
        return false;
    }

    List<Contacts.Pressing> pressings() {
        List<Contacts.Pressing> pressing = new ArrayList<>();
        List<Body> awake = awake();
        for (Body body : awake) {
            body.touched = false;
            body.carried = false;
        }
        for (Body body : awake) {
            for (Surroundings.Touch near : near(surroundings, body.at, body.reach(), Shot.TOUCHING_IN)) {
                pressing.add(new Contacts.Pressing(
                        body.contact(),
                        new Contacts.Against.Fixed(near.id(), near.surface(), near.velocity(), near.spin()),
                        near.normal(),
                        !body.touchingColliders.contains(near.id())));
                body.touched = true;
                body.carried = body.carried || near.moves();
            }
            boolean past = false;
            for (Body other : bodies) {
                if (other == body) {
                    past = true;
                    continue;
                }
                if (other.life == Life.GONE || (other.life == Life.AWAKE && !past)) {
                    continue;
                }
                if (!within(body.at, body, other.at, other, Shot.TOUCHING_IN)) {
                    continue;
                }
                Vec3 apart = body.at.minus(other.at);
                double distance = StrictMath.sqrt(apart.dot(apart));
                if (!(distance > 0)) {
                    continue;
                }
                Vec3 normal = apart.times(1 / distance);
                boolean fresh = !body.touchingBodies.contains(other.name);
                body.touched = true;
                if (other.life == Life.FIXED || other.life == Life.ASLEEP) {
                    pressing.add(new Contacts.Pressing(
                            body.contact(), new Contacts.Against.Fixed(other.name, Surface.PIECE), normal, fresh));
                } else {
                    other.touched = true;
                    pressing.add(new Contacts.Pressing(
                            body.contact(), new Contacts.Against.Ball(other.contact()), normal, fresh));
                }
            }
        }
        return pressing;
    }

    private List<Surroundings.Touch> near(Surroundings around, Vec3 at, Length reach, double within) {
        work++;
        return around.near(at, reach, within);
    }

    private List<Surroundings.Touch> near(Surroundings.Held part, Vec3 at, Length reach, double within) {
        work++;
        return part.near(at, reach, within);
    }

    private Surroundings placed(Surroundings around, double seconds) {
        work += around.placements();
        return around.at(seconds);
    }

    private List<Seen> wake() {
        List<Seen> woken = wokenByWhatMoves();
        List<Islands.Node> nodes = new ArrayList<>();
        for (Body body : bodies) {
            if (body.life == Life.GONE) {
                continue;
            }
            Islands.State state = body.life == Life.FIXED
                    ? Islands.State.HELD
                    : body.life == Life.ASLEEP
                            ? Islands.State.ASLEEP
                            : body.restFor == 0 ? Islands.State.MOVING : Islands.State.RESTING;
            nodes.add(new Islands.Node(body.name, state, body.at, body.radius(), body.touchingBodies));
        }
        Set<String> waking = Islands.toWake(nodes, Shot.TOUCHING_IN);
        for (Body body : bodies) {
            if (body.life == Life.ASLEEP && waking.contains(body.name)) {
                body.life = Life.AWAKE;
                body.restFor = 0;
                woken.add(body.seen());
            }
        }
        return woken;
    }

    private List<Seen> wokenByWhatMoves() {
        List<Body> woken = new ArrayList<>();
        for (Body body : bodies) {
            if (body.life == Life.ASLEEP && reachedByWhatMoves(body)) {
                body.life = Life.AWAKE;
                body.restFor = 0;
                woken.add(body);
            }
        }
        List<Seen> seen = new ArrayList<>();
        for (Body body : woken) {
            refresh(body);
            seen.add(body.seen());
        }
        return seen;
    }

    private boolean reachedByWhatMoves(Body body) {
        for (Surroundings.Held part : surroundings.moving()) {
            double within = Shot.TOUCHING_IN + part.sweep() * step;
            work++;
            if (part.reaches(body.at, body.radius() + within)) {
                for (Surroundings.Touch touch : near(part, body.at, body.reach(), within)) {
                    if (touch.moves()) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private Optional<Contacts.Resolved> resolve(List<Contacts.Pressing> pressing) {
        if (pressing.isEmpty()) {
            return Optional.empty();
        }
        Contacts.Resolved resolved = Contacts.resolve(surfaces, pressing);
        for (Contacts.Body moved : resolved.bodies()) {
            for (Body body : bodies) {
                if (body.name.equals(moved.name()) && body.life == Life.AWAKE) {
                    body.velocity = moved.velocity();
                    body.spin = moved.spin();
                }
            }
        }
        return Optional.of(resolved);
    }

    private void project() {
        List<Body> touched = new ArrayList<>();
        for (Body body : awake()) {
            if (body.touched) {
                touched.add(body);
            }
        }
        boolean moved = !touched.isEmpty();
        for (int pass = 0; pass < Shot.PROJECTIONS && moved; pass++) {
            moved = false;
            for (Body body : touched) {
                for (Surroundings.Touch near : near(surroundings, body.at, body.reach(), 0)) {
                    if (near.gap() < 0) {
                        body.at = body.at.along(near.normal(), -near.gap());
                        moved = true;
                    }
                }
                for (Body other : bodies) {
                    if (other == body || other.life == Life.GONE || !overlapping(body.at, body, other.at, other)) {
                        continue;
                    }
                    moved = true;
                    Vec3 apart = body.at.minus(other.at);
                    double distance = StrictMath.sqrt(apart.dot(apart));
                    double over = body.radius() + other.radius() - distance;
                    if (!(distance > 0) || !(over > 0)) {
                        continue;
                    }
                    Vec3 normal = apart.times(1 / distance);
                    double mine = body.inverseMass(), theirs = other.inverseMass();
                    double share = mine / (mine + theirs);
                    body.at = body.at.along(normal, over * share);
                    other.at = other.at.along(normal, -over * (1 - share));
                }
            }
        }
    }

    private List<Seen> settle(double covered) {
        List<Seen> slept = new ArrayList<>();
        for (Body body : awake()) {
            double moving = StrictMath.sqrt(body.velocity.dot(body.velocity))
                    + body.radius() * StrictMath.sqrt(body.spin.dot(body.spin));
            body.restFor =
                    body.touched && !body.carried && moving < Shot.REST_SPEED_IN_PER_S ? body.restFor + covered : 0;
            if (body.sleep == Sleep.WHEN_RESTING && body.restFor >= Shot.REST_SECONDS) {
                body.life = Life.ASLEEP;
                body.velocity = Vec3.zero();
                body.spin = Vec3.zero();
                slept.add(body.seen());
            }
        }
        return slept;
    }
}
