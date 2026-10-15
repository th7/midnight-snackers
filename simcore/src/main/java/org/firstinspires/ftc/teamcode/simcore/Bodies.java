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

    sealed interface Name permits Name.Given, Name.Numbered {
        String key();

        String text();

        static Checked<Name> of(String text) {
            if (text.isBlank()) {
                return Checked.rejected("a body has a name, not '" + text + "'");
            }
            return Checked.ok(new Given(text));
        }

        static Name launched() {
            return new Given(Shot.LAUNCHED);
        }

        static Name numbered(long number) {
            return new Numbered(number);
        }

        final class Given implements Name {
            private final String text;

            private Given(String text) {
                this.text = text;
            }

            @Override
            public String key() {
                return text;
            }

            @Override
            public String text() {
                return text;
            }

            @Override
            public boolean equals(Object other) {
                return other instanceof Given that && text.equals(that.text);
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

        record Numbered(long number) implements Name {
            @Override
            public String key() {
                StringBuilder blank = new StringBuilder();
                for (char digit : Long.toString(number).toCharArray()) {
                    blank.append(blankFor(digit));
                }
                return blank.toString();
            }

            private static char blankFor(char digit) {
                return switch (digit) {
                    case '0' -> '\u2000';
                    case '1' -> '\u2001';
                    case '2' -> '\u2002';
                    case '3' -> '\u2003';
                    case '4' -> '\u2004';
                    case '5' -> '\u2005';
                    case '6' -> '\u2006';
                    case '7' -> '\u2008';
                    case '8' -> '\u2009';
                    case '9' -> '\u200A';
                    default -> '\u3000';
                };
            }

            @Override
            public String text() {
                return "#" + number;
            }

            @Override
            public String toString() {
                return text();
            }
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

    static final class Allowance {
        private final long steps;
        private final long units;

        private Allowance(long steps, long units) {
            this.steps = steps;
            this.units = units;
        }

        static Allowance unlimited() {
            return new Allowance(1, Long.MAX_VALUE);
        }

        static Checked<Allowance> perWindow(long steps, long units) {
            if (steps < 1) {
                return Checked.rejected("a window of the engine's work is at least one step long, not " + steps);
            }
            if (units < 0) {
                return Checked.rejected("an engine is allowed no less than no work in a window, not " + units);
            }
            return Checked.ok(new Allowance(steps, units));
        }

        long windowOf(long step) {
            return (long) StrictMath.floor((double) step / steps);
        }

        long until(long began) {
            return units > Long.MAX_VALUE - began ? Long.MAX_VALUE : began + units;
        }
    }

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

    record Touched(Seen seen, Set<String> colliders, Set<String> bodies) {}

    record Around(List<Surroundings.Touch> touches, List<String> bodies) {}

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

    private record Leaning(Body one, Body other) {}

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
        private boolean squeezed;
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
                    entry.name().key(),
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
    private final Step stepping;
    private final double step;
    private final Optional<String> watching;
    private final List<Body> bodies;
    private Surroundings surroundings;
    private long steps;
    private double now;
    private long work;
    private long window;
    private long windowBegan;

    private Bodies(
            Surroundings surroundings,
            Air air,
            Surfaces surfaces,
            Step stepping,
            Optional<String> watching,
            List<Body> bodies,
            long steps,
            double now,
            long work,
            long window,
            long windowBegan) {
        this.surroundings = surroundings;
        this.air = air;
        this.surfaces = surfaces;
        this.stepping = stepping;
        this.step = stepping.seconds().value();
        this.watching = watching;
        this.bodies = bodies;
        this.steps = steps;
        this.now = now;
        this.work = work;
        this.window = window;
        this.windowBegan = windowBegan;
    }

    static Bodies watching(Surroundings surroundings, Air air, Surfaces surfaces, Step step, Name name) {
        return new Bodies(
                surroundings, air, surfaces, step, Optional.of(name.key()), new ArrayList<>(), 0, 0, 0, -1, 0);
    }

    static Bodies unwatched(Surroundings surroundings, Air air, Surfaces surfaces, Step step) {
        return new Bodies(surroundings, air, surfaces, step, Optional.empty(), new ArrayList<>(), 0, 0, 0, -1, 0);
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
        Bodies engine =
                new Bodies(surroundings, air, surfaces, step, Optional.of(Shot.LAUNCHED), bodies, 0, 0, 0, -1, 0);
        engine.touchingAtTheStart();
        return engine;
    }

    Bodies copy() {
        List<Body> copied = new ArrayList<>();
        for (Body body : bodies) {
            copied.add(body.copy());
        }
        return new Bodies(
                surroundings, air, surfaces, stepping, watching, copied, steps, now, work, window, windowBegan);
    }

    Surroundings surroundings() {
        return surroundings;
    }

    List<Seen> surroundedBy(Surroundings surroundings) {
        this.surroundings = placed(surroundings, time());
        List<Seen> woken = new ArrayList<>();
        for (Body body : reachedAhead(placed(surroundings, GridTime.at(steps + 1, stepping)))) {
            woken.add(body.seen());
        }
        touchingAtTheStart();
        return Collections.unmodifiableList(woken);
    }

    List<Seen> surroundedWithoutWorkWhileIdleBy(Surroundings surroundings) {
        long began = work;
        List<Seen> woken = surroundedBy(surroundings);
        if (!anyAwake()) {
            work = began;
        }
        return woken;
    }

    Optional<Seen> moved(String name, Motion motion) {
        return named(name).map(body -> {
            body.at = motion.at();
            body.velocity = motion.velocity();
            body.spin = motion.spin();
            body.start = new Kin(body.at, body.velocity, body.spin);
            body.trial = body.start;
            refresh(body);
            return body.seen();
        });
    }

    Checked<Seen> add(Name name, Ball shape, Motion motion, Start start) {
        if (named(name.key()).isPresent()) {
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

    boolean anyAwake() {
        for (Body body : bodies) {
            if (body.life == Life.AWAKE) {
                return true;
            }
        }
        return false;
    }

    List<Touched> touched() {
        List<Touched> touched = new ArrayList<>();
        for (Body body : bodies) {
            touched.add(new Touched(
                    body.seen(),
                    Collections.unmodifiableSet(body.touchingColliders),
                    Collections.unmodifiableSet(body.touchingBodies)));
        }
        return Collections.unmodifiableList(touched);
    }

    Optional<Around> around(String name) {
        return named(name).map(body -> {
            List<String> touching = new ArrayList<>();
            for (Body other : bodies) {
                if (other != body
                        && other.life != Life.GONE
                        && within(body.at, body, other.at, other, Shot.TOUCHING_IN)) {
                    touching.add(other.name);
                }
            }
            return new Around(
                    near(surroundings, body.at, body.reach(), Shot.TOUCHING_IN),
                    Collections.unmodifiableList(touching));
        });
    }

    Vec3 clearOf(Vec3 at, Length reach) {
        Vec3 cleared = at;
        double r = reach.inches();
        boolean moved = true;
        for (int pass = 0; pass < ShotSetup.SETTING_DOWN_PASSES && moved; pass++) {
            moved = false;
            for (Surroundings.Touch near : near(surroundings, cleared, reach, 0)) {
                if (near.gap() < -ShotSetup.SET_DOWN_IN) {
                    cleared = cleared.along(near.normal(), -near.gap());
                    moved = true;
                }
            }
            for (Body other : bodies) {
                Vec3 apart = cleared.minus(other.at);
                double distance = StrictMath.sqrt(apart.dot(apart));
                double over = r + other.radius() - distance;
                if (other.life != Life.GONE && distance > 0 && over > ShotSetup.SET_DOWN_IN) {
                    cleared = cleared.along(apart.times(1 / distance), over);
                    moved = true;
                }
            }
        }
        return cleared;
    }

    long steps() {
        return steps;
    }

    double now() {
        return now;
    }

    GridTime time() {
        return GridTime.at(steps, stepping);
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
        return step(Allowance.unlimited());
    }

    List<Span> step(Allowance allowance) {
        List<Span> spans = new ArrayList<>();
        long began = work;
        long thisWindow = allowance.windowOf(steps);
        if (thisWindow != window) {
            window = thisWindow;
            windowBegan = began;
        }
        long until = allowance.until(windowBegan);
        Surroundings ahead = placed(surroundings, GridTime.at(steps + 1, stepping));
        List<Seen> early = new ArrayList<>();
        for (Body body : reachedAhead(ahead)) {
            refresh(body);
            early.add(body.seen());
        }
        boolean moves = anyAwake();
        int pieces = piecesOfThisStep(ahead);
        for (int i = 1; i <= pieces; i++) {
            aPiece(i, pieces, until, spans, i == 1 ? early : List.of());
        }
        steps++;
        now = steps * step;
        if (!moves) {
            work = began;
        }
        return Collections.unmodifiableList(spans);
    }

    boolean idleThrough(long stepTarget) {
        long began = work;
        boolean idle = idlesThrough(stepTarget);
        work = began;
        return idle;
    }

    private boolean idlesThrough(long stepTarget) {
        if (stepTarget <= steps) {
            return stepTarget == steps;
        }
        for (Body body : bodies) {
            if (body.life == Life.AWAKE) {
                return false;
            }
        }
        double horizon = (stepTarget - steps) * step;
        Surroundings then = placed(surroundings, GridTime.at(stepTarget, stepping));
        for (Body body : bodies) {
            if (body.life == Life.ASLEEP
                    && (reachedByWhatMoves(body, surroundings, horizon) || reachedByWhatMoves(body, then, horizon))) {
                return false;
            }
        }
        steps = stepTarget;
        now = steps * step;
        surroundings = then;
        return true;
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

    private void aPiece(int piece, int pieces, long until, List<Span> spans, List<Seen> early) {
        double target = (steps + (double) piece / pieces) * step;
        double into = (double) (piece - 1) / pieces;
        int events = 0;
        List<Seen> pending = early;
        while (now < target) {
            List<Body> awake = awake();
            double span = target - now;
            for (Body body : awake) {
                body.start = new Kin(body.at, body.velocity, body.spin);
            }
            Optional<Glimpse> glimpse = watched().map(Glimpse::of);
            integrate(awake, span, into);
            double covered = span;
            if (events < Shot.MOST_EVENTS_PER_PIECE
                    && work < until
                    && overlapsSomethingNew(awake, (double) piece / pieces)) {
                double lo = 0, hi = 1;
                for (int i = 0; i < Shot.BISECTIONS; i++) {
                    double mid = (lo + hi) / 2;
                    double over = mid * span;
                    integrate(awake, over, into);
                    if (overlapsSomethingNew(awake, into + over / step)) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                integrate(awake, lo * span, into);
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
            into = covered == span ? (double) piece / pieces : into + covered / step;
            surroundings = placedPartWay(surroundings, into);
            List<Seen> woken = wake(pending);
            pending = List.of();
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

    private void integrate(List<Body> awake, double over, double into) {
        for (Body body : awake) {
            if (surroundings.squeezes()
                    && surroundings.squeezeReaches(body.start.at(), body.radius() + Shot.MOST_TRAVEL_IN)) {
                body.trial = squeezedRk4(body, body.start, over, into);
            } else {
                body.trial = rk4(body, body.start, over);
                work++;
            }
        }
    }

    private Kin squeezedRk4(Body body, Kin from, double dt, double into) {
        double half = into + dt / 2 / step;
        Kin k1 = squeezedRate(body, from, into);
        Kin k2 = squeezedRate(body, along(from, k1, dt / 2), half);
        Kin k3 = squeezedRate(body, along(from, k2, dt / 2), half);
        Kin k4 = squeezedRate(body, along(from, k3, dt), into + dt / step);
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

    private Kin squeezedRate(Body body, Kin state, double into) {
        Air.Motion moved = air.motion(body.shape, state.velocity(), state.spin());
        work++;
        List<Surroundings.Squeeze> squeezes = squeezing(body, state.at(), into);
        if (squeezes.isEmpty()) {
            return new Kin(state.velocity(), moved.acceleration(), moved.spinRate());
        }
        CompliantContact.Push pushed =
                CompliantContact.push(body.shape, state.velocity(), state.spin(), squeezes, stepping);
        return new Kin(
                state.velocity(),
                moved.acceleration().plus(pushed.acceleration()),
                moved.spinRate().plus(pushed.spinRate()));
    }

    private List<Surroundings.Squeeze> squeezing(Body body, Vec3 at, double into) {
        work++;
        GridTime then = GridTime.PartOfAStep.of(into)
                .map(part -> GridTime.partWay(steps, part, stepping))
                .orElse(GridTime.at(steps + 1, stepping));
        return surroundings.squeezingAt(then, at, body.reach());
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

    private boolean overlapsSomethingNew(List<Body> awake, double into) {
        if (awake.isEmpty()) {
            return false;
        }
        Surroundings then = placedPartWay(surroundings, into);
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
        List<Leaning> leaning = new ArrayList<>();
        List<Body> awake = awake();
        for (Body body : awake) {
            body.touched = false;
            body.squeezed = false;
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
            if (surroundings.squeezes() && surroundings.squeezeReaches(body.at, body.radius())) {
                work++;
                for (Surroundings.Squeeze squeeze : surroundings.squeezing(body.at, body.reach())) {
                    body.squeezed = true;
                    body.carried = body.carried || squeeze.moves();
                }
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
                    leaning.add(new Leaning(body, other));
                    pressing.add(new Contacts.Pressing(
                            body.contact(), new Contacts.Against.Ball(other.contact()), normal, fresh));
                }
            }
        }
        carriedThrough(leaning);
        return pressing;
    }

    private static void carriedThrough(List<Leaning> leaning) {
        boolean spread = true;
        while (spread) {
            spread = false;
            for (Leaning pair : leaning) {
                if (pair.one().carried != pair.other().carried) {
                    pair.one().carried = true;
                    pair.other().carried = true;
                    spread = true;
                }
            }
        }
    }

    private List<Surroundings.Touch> near(Surroundings around, Vec3 at, Length reach, double within) {
        work++;
        return around.near(at, reach, within);
    }

    private List<Surroundings.Touch> near(Surroundings.Held part, Vec3 at, Length reach, double within) {
        work++;
        return part.near(at, reach, within);
    }

    private Surroundings placedPartWay(Surroundings around, double into) {
        if (around.placements() == 0) {
            return around;
        }
        return placed(
                around,
                GridTime.PartOfAStep.of(into)
                        .map(part -> GridTime.partWay(steps, part, stepping))
                        .orElse(GridTime.at(steps + 1, stepping)));
    }

    private Surroundings placed(Surroundings around, GridTime time) {
        work += around.placements();
        return around.at(time);
    }

    private List<Seen> wake(List<Seen> early) {
        List<Seen> woken = new ArrayList<>(early);
        woken.addAll(wokenByWhatMoves());
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
            if (body.life == Life.ASLEEP && reachedByWhatMoves(body, surroundings, step)) {
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

    private List<Body> reachedAhead(Surroundings ahead) {
        List<Body> reached = new ArrayList<>();
        for (Body body : bodies) {
            if (body.life == Life.ASLEEP
                    && (reachedByWhatMoves(body, surroundings, step) || reachedByWhatMoves(body, ahead, step))) {
                body.life = Life.AWAKE;
                body.restFor = 0;
                reached.add(body);
            }
        }
        return reached;
    }

    private boolean reachedByWhatMoves(Body body, Surroundings around, double horizon) {
        for (Surroundings.Held part : around.moving()) {
            double within = Shot.TOUCHING_IN + part.sweep() * horizon;
            work++;
            if (part.reaches(body.at, body.radius() + within)) {
                for (Surroundings.Touch touch : near(part, body.at, body.reach(), within)) {
                    if (touch.moves()) {
                        return true;
                    }
                }
                if (part.squeezes() && squeezedByWhatMoves(part, body, within)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean squeezedByWhatMoves(Surroundings.Held part, Body body, double within) {
        work++;
        Length reach = Length.of(body.radius() + within).orElse(body.reach());
        for (Surroundings.Squeeze squeeze : part.squeezing(body.at, reach)) {
            if (squeeze.moves()) {
                return true;
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
            body.restFor = (body.touched || body.squeezed) && !body.carried && moving < Shot.REST_SPEED_IN_PER_S
                    ? body.restFor + covered
                    : 0;
            if (body.sleep == Sleep.WHEN_RESTING
                    && body.restFor >= Shot.REST_SECONDS
                    && !reachedByWhatMoves(body, surroundings, step)) {
                body.life = Life.ASLEEP;
                body.velocity = Vec3.zero();
                body.spin = Vec3.zero();
                slept.add(body.seen());
            }
        }
        return slept;
    }
}
