package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public final class Flight<K> {
    public static final double GRAVITY_IN_PER_S2 = 386.09;
    public static final double LANDING_SPEED_IN_PER_S = 25;
    public static final double BOUNCE = 0.3;
    public static final double FRICTION = 0.4;
    public static final double REST_SPEED_IN_PER_S = Shot.REST_SPEED_IN_PER_S;
    public static final double ROLL_SECONDS = 0.8;
    public static final double MOST_SPEED_IN_PER_S = 1000;
    public static final long STEP_NANOS = 100_000;
    public static final double NANOS_PER_SECOND = 1e9;
    public static final long WINDOW_STEPS = 50;
    public static final double WINDOW_SECONDS = WINDOW_STEPS * STEP_NANOS / NANOS_PER_SECOND;
    public static final long WINDOW_WORK = Shot.MOST_WORK_PER_SECOND * WINDOW_STEPS * STEP_NANOS / 1_000_000_000L;
    private static final double SIDE_ON = 0.5;
    private static final double CLEAR_WITHIN_IN = 0.02;

    public static final class Tick {
        public static final double MOST_SECONDS = 0.25;

        private final Seconds seconds;

        private Tick(Seconds seconds) {
            this.seconds = seconds;
        }

        public static Checked<Tick> of(Seconds seconds) {
            if (!(seconds.value() <= MOST_SECONDS)) {
                return Checked.rejected(
                        "a tick of flight is at most " + MOST_SECONDS + " s, not " + seconds.value() + " s");
            }
            return Checked.ok(new Tick(seconds));
        }

        public Seconds seconds() {
            return seconds;
        }
    }

    public sealed interface Landing<K> permits OnTheFloor, OutOfTheField {
        K ball();

        Vec3 at();
    }

    public record OnTheFloor<K>(K ball, Vec3 at, Vec2 velocity) implements Landing<K> {}

    public record OutOfTheField<K>(K ball, Vec3 at) implements Landing<K> {}

    public record Stepped<K>(Flight<K> flight, List<Landing<K>> landings) {}

    private record Physics(
            Colliders still,
            Surroundings around,
            Set<String> floors,
            Surfaces surfaces,
            Bodies.Allowance allowance,
            double pollenGrams,
            double nectarGrams,
            double inertia) {}

    private record Footing(boolean lands, Set<String> bodies) {
        static Footing none() {
            return new Footing(false, Collections.emptySet());
        }

        static Footing of(boolean lands, Set<String> bodies) {
            return new Footing(lands, Collections.unmodifiableSet(new HashSet<>(bodies)));
        }
    }

    private record Flying<K>(
            K key,
            Field.Kind kind,
            Length radius,
            String name,
            Optional<Field.Cell> cell,
            boolean presses,
            boolean awake,
            boolean onTheFloor,
            Optional<Footing> lying) {
        Flying<K> seen(Optional<Field.Cell> in, boolean pressing, boolean isAwake, boolean onFloor) {
            return new Flying<>(
                    key, kind, radius, name, in, pressing, isAwake, onFloor, isAwake ? Optional.empty() : lying);
        }

        Flying<K> lyingOn(Footing footing) {
            return new Flying<>(key, kind, radius, name, cell, presses, awake, onTheFloor, Optional.of(footing));
        }
    }

    private final Physics physics;
    private final Hives hives;
    private final Bodies bodies;
    private final List<Flying<K>> flying;
    private final long named;
    private final long nanos;

    private Flight(Physics physics, Hives hives, Bodies bodies, List<Flying<K>> flying, long named, long nanos) {
        this.physics = physics;
        this.hives = hives;
        this.bodies = bodies;
        this.flying = Collections.unmodifiableList(flying);
        this.named = named;
        this.nanos = nanos;
    }

    public static <K> Checked<Flight<K>> over(Field field) {
        return Hives.of(field).then(Flight::<K>over);
    }

    private static <K> Checked<Flight<K>> over(Hives hives) {
        ShotSettings defaults = ShotSettings.defaults();
        return Colliders.withoutHives(hives.field())
                .then(still -> Surroundings.of(still, List.of(hives.part()))
                        .then(around -> ShotSetup.air(defaults)
                                .then(air -> ShotSetup.surfaces(defaults)
                                        .then(surfaces -> stepping()
                                                .then(step -> Bodies.Allowance.perWindow(WINDOW_STEPS, WINDOW_WORK)
                                                        .map(allowance -> new Flight<K>(
                                                                new Physics(
                                                                        still,
                                                                        around,
                                                                        floorsOf(still),
                                                                        surfaces,
                                                                        allowance,
                                                                        defaults.value(ShotSetting.MASS),
                                                                        defaults.value(ShotSetting.NECTAR_MASS),
                                                                        defaults.value(ShotSetting.INERTIA)),
                                                                hives,
                                                                Bodies.unwatched(around, air, surfaces, step),
                                                                new ArrayList<>(),
                                                                0,
                                                                0)))))));
    }

    static Checked<Bodies.Step> stepping() {
        return Seconds.of(STEP_NANOS / NANOS_PER_SECOND).then(Bodies.Step::of);
    }

    private static Set<String> floorsOf(Colliders still) {
        Set<String> floors = new HashSet<>();
        for (Collider collider : still.all()) {
            if (collider.surface() == Surface.FLOOR) {
                floors.add(collider.name());
            }
        }
        return Collections.unmodifiableSet(floors);
    }

    public Hives hives() {
        return hives;
    }

    public Surfaces surfaces() {
        return physics.surfaces();
    }

    public long work() {
        return bodies.work();
    }

    private Checked<Ball> ballOf(Field.Kind kind, Length radius) {
        return Ball.of(
                radius, kind == Field.Kind.NECTAR ? physics.nectarGrams() : physics.pollenGrams(), physics.inertia());
    }

    public Checked<Flight<K>> with(K ball, Field.Kind kind, Length radius, Vec3 at, Vec3 velocity) {
        double speed = StrictMath.sqrt(velocity.dot(velocity));
        if (!(speed <= MOST_SPEED_IN_PER_S)) {
            return Checked.rejected(
                    "a ball comes into flight at most " + MOST_SPEED_IN_PER_S + " in/s, not at " + velocity);
        }
        return Bodies.Motion.of(at, velocity, Vec3.zero())
                .then(motion -> ballOf(kind, radius).map(shape -> {
                    Work<K> work = new Work<>(this);
                    work.catchUp();
                    work.remove(ball);
                    work.add(ball, kind, radius, shape, motion, new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING));
                    return work.checked();
                }));
    }

    public Checked<Flight<K>> restingAt(K ball, Field.Kind kind, Length radius, Vec3 at) {
        return Bodies.Motion.of(at, Vec3.zero(), Vec3.zero())
                .then(placed -> ballOf(kind, radius).then(shape -> {
                    Work<K> work = new Work<>(this);
                    work.catchUp();
                    work.remove(ball);
                    return Bodies.Motion.of(work.bodies.clearOf(placed.at(), radius), Vec3.zero(), Vec3.zero())
                            .map(cleared -> {
                                work.add(ball, kind, radius, shape, cleared, new Bodies.Start.Asleep());
                                return work.checked();
                            });
                }));
    }

    public Checked<Flight<K>> within(K ball, Field.Kind kind, Length radius, Field.Cell cell) {
        return ballOf(kind, radius).then(shape -> {
            Work<K> work = new Work<>(this);
            work.catchUp();
            work.remove(ball);
            for (Vec3 spot : work.hives.restingSpots(cell, radius)) {
                if (work.clear(spot, radius)) {
                    return Bodies.Motion.of(spot, Vec3.zero(), Vec3.zero()).map(motion -> {
                        work.add(ball, kind, radius, shape, motion, new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING));
                        return work.checked();
                    });
                }
            }
            return Checked.rejected("there is no room left in " + cell.name() + " for another " + kind.named());
        });
    }

    public Flight<K> without(K ball) {
        if (!holds(ball)) {
            return this;
        }
        Work<K> work = new Work<>(this);
        work.catchUp();
        work.remove(ball);
        return work.checked();
    }

    public boolean holds(K ball) {
        return flyingOf(ball).isPresent();
    }

    private Optional<Flying<K>> flyingOf(K ball) {
        for (Flying<K> one : flying) {
            if (one.key().equals(ball)) {
                return Optional.of(one);
            }
        }
        return Optional.empty();
    }

    public Optional<Vec3> at(K ball) {
        return flyingOf(ball).flatMap(one -> bodies.body(one.name())).map(Bodies.Seen::at);
    }

    public Optional<Vec3> velocity(K ball) {
        return flyingOf(ball).flatMap(one -> bodies.body(one.name())).map(Bodies.Seen::velocity);
    }

    public int fill(Field.Hive hive) {
        Optional<Field.Cell> up = hives.upturnedCell(hive);
        int fill = 0;
        for (Flying<K> one : flying) {
            if (one.cell().isPresent() && one.cell().equals(up)) {
                fill += Hives.fills(one.kind());
            }
        }
        return fill;
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
        for (Flying<K> one : flying) {
            one.cell().ifPresent(cell -> out.merge(cell.alliance(), 1, Integer::sum));
        }
        return out;
    }

    public Stepped<K> step(Tick tick) {
        long more = StrictMath.round(tick.seconds().value() * NANOS_PER_SECOND);
        long then = more > Long.MAX_VALUE - nanos ? Long.MAX_VALUE : nanos + more;
        Work<K> work = new Work<>(this);
        work.tipWhatIsFull();
        if (!work.bodies.anyAwake() && !work.hives.moving()) {
            return new Stepped<>(work.at(then), List.of());
        }
        work.advanceTo(stepsTo(then));
        return new Stepped<>(work.at(then), Collections.unmodifiableList(work.landings));
    }

    private static long stepsTo(long nanos) {
        long whole = nanos / 100_000;
        return whole * STEP_NANOS < nanos ? whole + 1 : whole;
    }

    private static final class Work<K> {
        private final Physics physics;
        private final List<Flying<K>> flying;
        private final List<Landing<K>> landings = new ArrayList<>();
        private final long nanos;
        private Hives hives;
        private Bodies bodies;
        private long named;
        private boolean copied;

        Work(Flight<K> flight) {
            this.physics = flight.physics;
            this.flying = new ArrayList<>(flight.flying);
            this.nanos = flight.nanos;
            this.hives = flight.hives;
            this.bodies = flight.bodies;
            this.named = flight.named;
        }

        Flight<K> at(long then) {
            return new Flight<>(physics, hives, bodies, flying, named, then);
        }

        Flight<K> checked() {
            seeAll();
            tipWhatIsFull();
            return at(nanos);
        }

        private Bodies engine() {
            if (!copied) {
                bodies = bodies.copy();
                copied = true;
            }
            return bodies;
        }

        void catchUp() {
            Bodies engine = engine();
            if (engine.steps() < stepsTo(nanos)) {
                advanceTo(stepsTo(nanos));
            }
        }

        void advanceTo(long target) {
            Bodies engine = engine();
            while (engine.steps() < target) {
                boolean wasMoving = hives.moving();
                if (!engine.anyAwake() && engine.idleThrough(target)) {
                    hives = hives.at(engine.time());
                    seeAll();
                    continue;
                }
                engine.step(physics.allowance());
                hives = hives.at(engine.time());
                afterAStep(wasMoving || hives.moving());
            }
        }

        void remove(K ball) {
            Optional<Flying<K>> gone = Optional.empty();
            for (Flying<K> one : flying) {
                if (one.key().equals(ball)) {
                    gone = Optional.of(one);
                }
            }
            gone.ifPresent(one -> {
                engine().remove(one.name());
                flying.remove(one);
            });
        }

        void add(K ball, Field.Kind kind, Length radius, Ball shape, Bodies.Motion motion, Bodies.Start start) {
            named++;
            Bodies.Name name = Bodies.Name.numbered(named);
            engine().add(name, shape, motion, start)
                    .fold(
                            seen -> flying.add(new Flying<>(
                                    ball,
                                    kind,
                                    radius,
                                    name.text(),
                                    Optional.empty(),
                                    false,
                                    start.life() == Bodies.Life.AWAKE,
                                    false,
                                    Optional.empty())),
                            rule -> false);
        }

        boolean clear(Vec3 spot, Length radius) {
            for (Bodies.Seen other : engine().seen()) {
                Vec3 apart = other.at().minus(spot);
                double reach = radiusOf(other.name()) + radius.inches() - CLEAR_WITHIN_IN;
                if (apart.dot(apart) < reach * reach) {
                    return false;
                }
            }
            return true;
        }

        private double radiusOf(String name) {
            for (Flying<K> one : flying) {
                if (one.name().equals(name)) {
                    return one.radius().inches();
                }
            }
            return 0;
        }

        void seeAll() {
            see(engine().touched(), true);
        }

        private void see(List<Bodies.Touched> touched, boolean all) {
            Map<String, Bodies.Touched> byName = new HashMap<>();
            for (Bodies.Touched one : touched) {
                byName.put(one.seen().name(), one);
            }
            Map<String, Optional<Field.Cell>> cells = new HashMap<>();
            for (Flying<K> one : flying) {
                Bodies.Touched now = byName.get(one.name());
                if (now != null && (all || one.awake() || now.seen().life() == Bodies.Life.AWAKE)) {
                    cells.put(one.name(), hives.cellHolding(now.seen().at()));
                } else {
                    cells.put(one.name(), one.cell());
                }
            }
            List<Flying<K>> seen = new ArrayList<>();
            for (Flying<K> one : flying) {
                Bodies.Touched now = byName.get(one.name());
                if (now == null || !(all || one.awake() || now.seen().life() == Bodies.Life.AWAKE)) {
                    seen.add(one);
                    continue;
                }
                Optional<Field.Cell> cell = cells.getOrDefault(one.name(), Optional.empty());
                boolean awake = now.seen().life() == Bodies.Life.AWAKE;
                seen.add(one.seen(cell, presses(cell, now, awake, cells), awake, onTheFloor(one, now, awake)));
            }
            flying.clear();
            flying.addAll(seen);
        }

        private boolean presses(
                Optional<Field.Cell> cell, Bodies.Touched now, boolean awake, Map<String, Optional<Field.Cell>> cells) {
            if (cell.isEmpty()) {
                return false;
            }
            if (!awake) {
                return true;
            }
            for (String collider : now.colliders()) {
                if (hives.cellOf(collider).equals(cell)) {
                    return true;
                }
            }
            for (String body : now.bodies()) {
                if (cells.getOrDefault(body, Optional.empty()).equals(cell)) {
                    return true;
                }
            }
            return false;
        }

        private boolean onTheFloor(Flying<K> one, Bodies.Touched now, boolean awake) {
            if (awake) {
                for (String collider : now.colliders()) {
                    if (physics.floors().contains(collider)) {
                        return true;
                    }
                }
                return false;
            }
            return now.seen().at().z() - one.radius().inches() <= Shot.TOUCHING_IN;
        }

        void tipWhatIsFull() {
            for (Field.Hive hive : hives.field().hives()) {
                if (hives.tipping(hive)) {
                    continue;
                }
                Optional<Field.Cell> up = hives.upturnedCell(hive);
                int fill = 0;
                for (Flying<K> one : flying) {
                    if (one.presses() && one.cell().isPresent() && one.cell().equals(up)) {
                        fill += Hives.fills(one.kind());
                    }
                }
                if (fill >= Hives.FULL) {
                    tip(hive);
                }
            }
        }

        private void tip(Field.Hive hive) {
            catchUp();
            Bodies engine = engine();
            hives = hives.at(engine.time()).tipped(hive);
            engine.surroundedBy(physics.around().replacing(hives.part()));
            seeAll();
        }

        private void afterAStep(boolean hivesMoved) {
            List<Bodies.Touched> touched = engine().touched();
            goOut(touched);
            land(touched);
            see(engine().touched(), hivesMoved);
            tipWhatIsFull();
        }

        private void goOut(List<Bodies.Touched> touched) {
            for (Bodies.Touched one : touched) {
                if (one.seen().life() != Bodies.Life.AWAKE) {
                    continue;
                }
                Vec3 at = one.seen().at();
                if (physics.still().outside(at).isPresent()) {
                    named(one.seen().name()).ifPresent(gone -> {
                        landings.add(new OutOfTheField<>(
                                gone.key(),
                                new Vec3(at.x(), at.y(), gone.radius().inches())));
                        engine().remove(gone.name());
                        flying.remove(gone);
                    });
                }
            }
        }

        private Optional<Flying<K>> named(String name) {
            for (Flying<K> one : flying) {
                if (one.name().equals(name)) {
                    return Optional.of(one);
                }
            }
            return Optional.empty();
        }

        private void land(List<Bodies.Touched> touched) {
            Map<String, Footing> footings = new LinkedHashMap<>();
            Map<String, Bodies.Seen> seen = new HashMap<>();
            for (Bodies.Touched now : touched) {
                named(now.seen().name()).ifPresent(ball -> {
                    Footing footing = footingOf(ball, now);
                    if (footing.lands()) {
                        footings.put(ball.name(), footing);
                        seen.put(ball.name(), now.seen());
                    }
                });
            }
            boolean pruned = true;
            while (pruned) {
                pruned = false;
                for (Map.Entry<String, Footing> footing : new ArrayList<>(footings.entrySet())) {
                    if (!footings.keySet().containsAll(footing.getValue().bodies())) {
                        footings.remove(footing.getKey());
                        pruned = true;
                    }
                }
            }
            for (Flying<K> one : new ArrayList<>(flying)) {
                Bodies.Seen landed = seen.get(one.name());
                if (landed != null && footings.containsKey(one.name())) {
                    Vec3 at = landed.at(), velocity = landed.velocity();
                    landings.add(new OnTheFloor<>(
                            one.key(),
                            new Vec3(at.x(), at.y(), one.radius().inches()),
                            new Vec2(velocity.x(), velocity.y())));
                    engine().remove(one.name());
                    flying.remove(one);
                }
            }
        }

        private Footing footingOf(Flying<K> ball, Bodies.Touched now) {
            Bodies.Seen seen = now.seen();
            if (seen.life() == Bodies.Life.ASLEEP) {
                if (ball.lying().isPresent()) {
                    return ball.lying().orElse(Footing.none());
                }
                Footing footing = seen.at().z() - ball.radius().inches() <= Shot.TOUCHING_IN
                        ? footingAround(ball, seen)
                        : Footing.none();
                replace(ball, ball.lyingOn(footing));
                return footing;
            }
            if (seen.life() != Bodies.Life.AWAKE || !ball.onTheFloor() || !onTheFloor(ball, now, true)) {
                return Footing.none();
            }
            Vec3 v = seen.velocity(), w = seen.spin();
            double r = ball.radius().inches();
            double slipX = v.x() - r * w.y(), slipY = v.y() + r * w.x();
            if (!(StrictMath.abs(v.z()) <= physics.surfaces().threshold()
                    && StrictMath.sqrt(slipX * slipX + slipY * slipY) <= REST_SPEED_IN_PER_S)) {
                return Footing.none();
            }
            if (physics.floors().containsAll(now.colliders())) {
                return Footing.of(true, now.bodies());
            }
            return footingAround(ball, seen);
        }

        private Footing footingAround(Flying<K> ball, Bodies.Seen seen) {
            return engine().around(seen.name())
                    .map(around -> {
                        boolean onTheFloor = false, besides = true;
                        for (Surroundings.Touch touch : around.touches()) {
                            if (physics.floors().contains(touch.id())) {
                                onTheFloor = true;
                            } else {
                                besides = besides && besideIt(ball, touch);
                            }
                        }
                        return Footing.of(onTheFloor && besides, new HashSet<>(around.bodies()));
                    })
                    .orElse(Footing.none());
        }

        private boolean besideIt(Flying<K> ball, Surroundings.Touch touch) {
            if (touch.surface() == Surface.WALL) {
                return true;
            }
            return StrictMath.abs(touch.normal().z()) <= SIDE_ON
                    && hives.field()
                            .obstacle(touch.id())
                            .map(obstacle ->
                                    obstacle.clears() < 2 * ball.radius().inches())
                            .orElse(false);
        }

        private void replace(Flying<K> was, Flying<K> now) {
            List<Flying<K>> replaced = new ArrayList<>();
            for (Flying<K> one : flying) {
                replaced.add(one.name().equals(was.name()) ? now : one);
            }
            flying.clear();
            flying.addAll(replaced);
        }
    }
}
