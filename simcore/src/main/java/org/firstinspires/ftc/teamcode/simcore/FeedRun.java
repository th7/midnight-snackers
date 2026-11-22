package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

final class FeedRun {
    private static final double OPEN_FIELD_IN = 144;
    private static final double OPEN_WALL_IN = 12;

    private record Parts(Spindexer spindexer, Launcher launcher, Bodies.Step step) {}

    private record Engine(Bodies bodies, Length radius, Ball ball, Bodies.Allowance allowance, Seconds tick) {}

    private final Feed.Plan plan;
    private final Launcher launcher;
    private final Engine engine;
    private final Bodies bodies;
    private final long tickSteps;
    private final long jammedTicks;
    private final Map<String, Boolean> inTheLauncher;
    private final List<Feed.Frame> frames;
    private final List<Feed.Event> events;
    private Spindexer spindexer;
    private Spindexer.Spinner.Turning turning;
    private Spindexer.RampAngle ramp;
    private Spindexer.Loads loads;
    private long stalledTicks;

    private FeedRun(Feed.Plan plan, Parts parts, Engine engine, Map<String, Boolean> inTheLauncher) {
        this.plan = plan;
        this.spindexer = parts.spindexer();
        this.launcher = parts.launcher();
        this.engine = engine;
        this.bodies = engine.bodies();
        this.tickSteps =
                StrictMath.round(engine.tick().value() / parts.step().seconds().value());
        this.jammedTicks =
                StrictMath.round(Feed.JAMMED_FOR_SECONDS / engine.tick().value());
        this.inTheLauncher = inTheLauncher;
        this.frames = new ArrayList<>();
        this.events = new ArrayList<>();
        this.turning = Spindexer.Spinner.Turning.still();
        this.ramp = plan.ramp().down();
        this.loads = Spindexer.Loads.none();
    }

    static Checked<FeedRun> starting(Feed.Plan plan) {
        return parts(plan).then(parts -> engine(parts).then(engine -> seated(plan, parts, engine)));
    }

    private static Checked<Parts> parts(Feed.Plan plan) {
        return Flight.stepping()
                .then(step -> Launcher.Frame.of(0, 0, 0)
                        .then(frame -> Spindexer.placed(
                                        plan.shape(),
                                        frame,
                                        Spindexer.Motion.still(
                                                GridTime.at(0, step),
                                                Spindexer.Spinner.Turning.still(),
                                                plan.ramp().down()))
                                .then(spindexer -> Launcher.Faces.fedByARamp()
                                        .then(faces -> Launcher.Course.of(
                                                        frame, GridTime.at(0, step), frame, GridTime.at(0, step))
                                                .map(still -> Launcher.placed(faces, still, plan.flywheel())))
                                        .map(launcher -> new Parts(spindexer, launcher, step)))));
    }

    private static Checked<Engine> engine(Parts parts) {
        ShotSettings defaults = ShotSettings.defaults();
        Map<ShotSetting, Double> spindexers = new EnumMap<>(ShotSetting.class);
        spindexers.put(ShotSetting.FRAME_FRICTION, Feed.FRICTION);
        spindexers.put(ShotSetting.FRAME_BOUNCE, Feed.RESTITUTION);
        return openAir()
                .then(still -> Surroundings.of(still, List.of(parts.spindexer(), parts.launcher())))
                .then(around -> ShotSetup.air(defaults)
                        .then(air -> ShotSettings.of(spindexers)
                                .then(settings -> ShotSetup.surfaces(settings))
                                .map(surfaces -> Bodies.unwatched(around, air, surfaces, parts.step()))))
                .then(bodies -> Length.of(Launcher.POLLEN_RADIUS_IN)
                        .then(radius -> Ball.of(
                                        radius, defaults.value(ShotSetting.MASS), defaults.value(ShotSetting.INERTIA))
                                .then(ball -> Bodies.Allowance.perWindow(Flight.WINDOW_STEPS, Flight.WINDOW_WORK)
                                        .then(allowance -> Seconds.of(Shot.FRAME_SECONDS)
                                                .map(tick -> new Engine(bodies, radius, ball, allowance, tick))))));
    }

    private static Checked<Colliders> openAir() {
        return Length.of(OPEN_FIELD_IN)
                .then(size -> Length.of(OPEN_WALL_IN)
                        .then(wall ->
                                Field.of(size, wall, List.of(), List.of(), List.of(), List.of(), List.of(), List.of())))
                .map(field -> Colliders.nothingIn(field));
    }

    private static Checked<FeedRun> seated(Feed.Plan plan, Parts parts, Engine engine) {
        Map<String, Boolean> inTheLauncher = new LinkedHashMap<>();
        List<Vec3> placed = new ArrayList<>();
        Length radius = engine.radius();
        long number = 0;
        for (Feed.Pollen pollen : plan.pollen()) {
            number++;
            String name = "Pollen " + number;
            Vec3 at = parts.spindexer().againstTheWall(radius, pollen.degreesFromTheBack());
            for (Surroundings.Touch touch : parts.spindexer().near(at, radius, 0)) {
                if (touch.gap() < -Shot.TOUCHING_IN) {
                    return Checked.rejected("a pollen starts clear of the spindexer, and " + name + ", " + pollen
                            + ", overlaps " + touch.id() + " by " + -touch.gap() + " in");
                }
            }
            for (Vec3 other : placed) {
                Vec3 apart = at.minus(other);
                double over = 2 * radius.inches() - StrictMath.sqrt(apart.dot(apart));
                if (over > Shot.TOUCHING_IN) {
                    return Checked.rejected("a pollen starts clear of the others, and " + name + ", " + pollen
                            + ", overlaps one by " + over + " in");
                }
            }
            placed.add(at);
            Checked<Bodies.Seen> added = Bodies.Name.of(name)
                    .then(named -> Bodies.Motion.of(at, Vec3.zero(), Vec3.zero())
                            .then(motion -> engine.bodies()
                                    .add(
                                            named,
                                            engine.ball(),
                                            motion,
                                            new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING))));
            if (added instanceof Checked.Rejected<Bodies.Seen> rejected) {
                return Checked.rejected(rejected.rule());
            }
            inTheLauncher.put(name, false);
        }
        engine.bodies().tallying(Spindexer.tallied());
        FeedRun run = new FeedRun(plan, parts, engine, inTheLauncher);
        run.takeAFrame();
        return Checked.ok(run);
    }

    Feed.Fed finished() {
        Optional<Feed.Finish> finish = finish();
        while (finish.isEmpty()) {
            aTick();
            takeAFrame();
            finish = finish();
        }
        return new Feed.Fed(
                Collections.unmodifiableList(frames),
                Collections.unmodifiableList(events),
                finish.orElse(new Feed.Finish.OutOfTime(seconds(bodies.now()))),
                bodies.work(),
                seconds(bodies.now()),
                new Feed.Drawing(spindexer.axes(), spindexer.measures(), launcher.axes(), launcher.measures()));
    }

    private Optional<Feed.Finish> finish() {
        double now = bodies.now();
        Seconds at = seconds(now);
        if (!inTheLauncher.isEmpty() && bodies.seen().isEmpty()) {
            return Optional.of(new Feed.Finish.AllLeft(at));
        }
        if (plan.budget().spentBy(bodies.work(), at)) {
            return Optional.of(new Feed.Finish.TooMuchWork(at));
        }
        boolean stillFromNow = plan.spinnerPower().value() == 0
                && plan.ramp().stillFrom(now, plan.length())
                && ramp.equals(plan.ramp().commandedAt(now));
        if (stillFromNow && !bodies.anyAwake()) {
            return Optional.of(new Feed.Finish.AtRest(at));
        }
        if (now >= plan.length().value() - engine.tick().value() / 2) {
            return Optional.of(new Feed.Finish.OutOfTime(at));
        }
        return Optional.empty();
    }

    private void aTick() {
        Seconds tick = engine.tick();
        double now = bodies.now();
        Spindexer.RampAngle nextRamp = plan.rampServo().next(ramp, plan.ramp().commandedAt(now), loads.ramp(), tick);
        Power power = plan.spinnerPower();
        Spindexer.Torque against = power.value() < 0 ? loads.spinner().negated() : loads.spinner();
        Spindexer.Spinner.Turning nextTurning = plan.spinner().next(turning, power, against, tick);
        Spindexer moved = Spindexer.Motion.of(bodies.time(), tick, turning, nextTurning, ramp, nextRamp)
                .map(motion -> spindexer.moved(motion))
                .orElse(spindexer);
        bodies.surroundings()
                .replacing(moved)
                .fold(
                        around -> {
                            bodies.surroundedWithoutWorkWhileIdleBy(around);
                            return true;
                        },
                        rule -> false);
        spindexer = moved;
        long target = bodies.steps() + tickSteps;
        while (bodies.steps() < target) {
            if (!bodies.anyAwake() && bodies.idleThrough(target)) {
                continue;
            }
            bodies.step(engine.allowance());
            watchTheLauncher();
        }
        loads = spindexer.loads(bodies.takeTallies(), tick);
        watchTheSpinner(
                power,
                nextTurning,
                plan.spinner().next(turning, power, Spindexer.Loads.none().spinner(), tick));
        turning = nextTurning;
        ramp = nextRamp;
        fallingOut();
    }

    private void watchTheSpinner(Power power, Spindexer.Spinner.Turning turned, Spindexer.Spinner.Turning unloaded) {
        boolean stalled = power.value() != 0
                && StrictMath.abs(turned.revolutionsPerSecond())
                        < Feed.JAMMED_BELOW_UNLOADED * StrictMath.abs(unloaded.revolutionsPerSecond());
        stalledTicks = stalled ? stalledTicks + 1 : 0;
        if (stalledTicks == jammedTicks) {
            List<String> pinned = new ArrayList<>();
            for (Bodies.Seen pollen : bodies.seen()) {
                if (spindexer.touchesTheSpinner(pollen.at(), engine.radius())) {
                    pinned.add(pollen.name());
                }
            }
            events.add(new Feed.Event.Jammed(seconds(bodies.now()), Collections.unmodifiableList(pinned)));
        }
    }

    private void watchTheLauncher() {
        Launcher then = launcher.at(bodies.time());
        Spindexer spindexerThen = spindexer.at(bodies.time());
        Length radius = engine.radius();
        for (Bodies.Seen pollen : bodies.seen()) {
            if (pollen.life() != Bodies.Life.AWAKE) {
                continue;
            }
            boolean touching = !then.squeezing(pollen.at(), radius).isEmpty()
                    || !then.near(pollen.at(), radius, Shot.TOUCHING_IN).isEmpty();
            boolean was = inTheLauncher.getOrDefault(pollen.name(), false);
            if (was && !touching && !spindexerThen.reaches(pollen.at(), radius.inches())) {
                events.add(new Feed.Event.Thrown(
                        pollen.name(), seconds(bodies.now()), pollen.at(), pollen.velocity(), pollen.spin()));
                bodies.remove(pollen.name());
            }
            inTheLauncher.put(pollen.name(), touching);
        }
    }

    private void fallingOut() {
        Length radius = engine.radius();
        for (Bodies.Seen pollen : bodies.seen()) {
            if (!spindexer.reaches(pollen.at(), radius.inches()) && !launcher.reaches(pollen.at(), radius.inches())) {
                events.add(new Feed.Event.FellOut(pollen.name(), seconds(bodies.now()), pollen.at()));
                bodies.remove(pollen.name());
            }
        }
    }

    private void takeAFrame() {
        List<Feed.Seen> pollen = new ArrayList<>();
        for (Bodies.Seen body : bodies.seen()) {
            pollen.add(new Feed.Seen(body.name(), body.at(), body.velocity(), body.spin(), body.turn()));
        }
        frames.add(new Feed.Frame(
                seconds(bodies.now()),
                StrictMath.toDegrees(turning.radians()),
                turning.revolutionsPerSecond(),
                loads.spinner().newtonMetres(),
                ramp.degrees(),
                loads.ramp().newtonMetres(),
                Collections.unmodifiableList(pollen)));
    }

    private static Seconds seconds(double t) {
        return Seconds.of(t).orElse(Seconds.zero());
    }
}
