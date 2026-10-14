package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Shot.Crossed;
import org.firstinspires.ftc.teamcode.simcore.Shot.Fate;
import org.firstinspires.ftc.teamcode.simcore.Shot.Finish;
import org.firstinspires.ftc.teamcode.simcore.Shot.Finished;
import org.firstinspires.ftc.teamcode.simcore.Shot.FirstBounce;
import org.firstinspires.ftc.teamcode.simcore.Shot.Flying;
import org.firstinspires.ftc.teamcode.simcore.Shot.Frame;
import org.firstinspires.ftc.teamcode.simcore.Shot.Impact;
import org.firstinspires.ftc.teamcode.simcore.Shot.Moving;
import org.firstinspires.ftc.teamcode.simcore.Shot.Phase;
import org.firstinspires.ftc.teamcode.simcore.Shot.Placed;
import org.firstinspires.ftc.teamcode.simcore.Shot.Resting;
import org.firstinspires.ftc.teamcode.simcore.Shot.Story;
import org.firstinspires.ftc.teamcode.simcore.Shot.Track;

final class ShotRun {
    private static final class History {
        private final World.Piece piece;
        private final List<Placed> samples;
        private Fate fate;

        private History(World.Piece piece, List<Placed> samples, Fate fate) {
            this.piece = piece;
            this.samples = new ArrayList<>(samples);
            this.fate = fate;
        }

        History copy() {
            return new History(piece, samples, fate);
        }
    }

    private final World world;
    private final Ball pollen;
    private final WheelPair.Launched launched;
    private final double step;
    private final Bodies bodies;
    private final Map<String, History> pieces;
    private final List<Frame> frames;
    private final List<Impact> impacts;
    private final List<Crossed> crossings;
    private long framesTaken;
    private Bodies.Seen ball;
    private Frame apex;
    private Optional<FirstBounce> firstBounce;
    private Phase phase;
    private List<Contacts.Pressing> pressingThePollen;

    private ShotRun(
            World world,
            Ball pollen,
            WheelPair.Launched launched,
            Bodies bodies,
            Map<String, History> pieces,
            List<Frame> frames,
            List<Impact> impacts,
            List<Crossed> crossings,
            long framesTaken,
            Bodies.Seen ball,
            Frame apex,
            Optional<FirstBounce> firstBounce,
            Phase phase,
            List<Contacts.Pressing> pressingThePollen) {
        this.world = world;
        this.pollen = pollen;
        this.launched = launched;
        this.step = world.step().value();
        this.bodies = bodies;
        this.pieces = new LinkedHashMap<>();
        for (Map.Entry<String, History> piece : pieces.entrySet()) {
            this.pieces.put(piece.getKey(), piece.getValue().copy());
        }
        this.frames = new ArrayList<>(frames);
        this.impacts = new ArrayList<>(impacts);
        this.crossings = new ArrayList<>(crossings);
        this.framesTaken = framesTaken;
        this.ball = ball;
        this.apex = apex;
        this.firstBounce = firstBounce;
        this.phase = phase;
        this.pressingThePollen = new ArrayList<>(pressingThePollen);
    }

    static ShotRun starting(World world, Ball pollen, WheelPair.Launched launched) {
        Bodies bodies = Bodies.launchedAmong(
                Surroundings.still(world.colliders()),
                world.air(),
                world.surfaces(),
                world.stepping(),
                pollen,
                Bodies.Motion.of(launched),
                world.bodies());
        Map<String, History> pieces = new LinkedHashMap<>();
        for (World.Piece piece : world.pieces()) {
            pieces.put(
                    piece.name(),
                    new History(
                            piece,
                            List.of(new Placed(Seconds.zero(), piece.at(), Turn.identity())),
                            new Fate.OnTheField()));
        }
        Bodies.Seen ball = new Bodies.Seen(
                Shot.LAUNCHED,
                launched.at(),
                launched.velocity(),
                launched.spin(),
                Turn.identity(),
                Bodies.Life.AWAKE,
                0);
        ShotRun run = new ShotRun(
                world,
                pollen,
                launched,
                bodies,
                pieces,
                List.of(),
                List.of(),
                List.of(),
                0,
                ball,
                new Frame(Seconds.zero(), moving(ball, Phase.FLYING)),
                Optional.empty(),
                Phase.FLYING,
                List.of());
        run.phase = run.phaseOfThePollen(bodies.pressings(), ball);
        run.apex = run.frameNow();
        run.takeAFrame();
        return run;
    }

    ShotRun copy() {
        return new ShotRun(
                world,
                pollen,
                launched,
                bodies.copy(),
                pieces,
                frames,
                impacts,
                crossings,
                framesTaken,
                ball,
                apex,
                firstBounce,
                phase,
                pressingThePollen);
    }

    Story story() {
        List<Track> tracks = new ArrayList<>();
        for (History history : pieces.values()) {
            tracks.add(new Track(history.piece, Collections.unmodifiableList(history.samples), history.fate));
        }
        return new Story(
                world,
                pollen,
                launched,
                Collections.unmodifiableList(frames),
                Collections.unmodifiableList(impacts),
                Collections.unmodifiableList(crossings),
                apex,
                firstBounce,
                Collections.unmodifiableList(tracks),
                bodies.work(),
                seconds(bodies.now()));
    }

    private static Seconds seconds(double t) {
        return Seconds.of(t).orElse(Seconds.zero());
    }

    private static Moving moving(Bodies.Seen ball, Phase phase) {
        return new Moving(ball.at(), ball.velocity(), ball.spin(), ball.turn(), phase);
    }

    Shot until(double until) {
        while (bodies.steps() * step < until) {
            Optional<Finish> finish = aStep();
            if (finish.isPresent()) {
                return finish.<Shot>map(this::finished).orElse(new Flying(this));
            }
        }
        return new Flying(this);
    }

    private Finished finished(Finish finish) {
        Frame last = frameNow();
        boolean taken = false;
        for (Frame frame : frames) {
            taken = frame.t().equals(last.t());
        }
        if (!taken) {
            frames.add(last);
        }
        sampleTheAwake();
        return new Finished(copy().story(), finish);
    }

    private Optional<Finish> aStep() {
        for (Bodies.Span span : bodies.step()) {
            replay(span);
        }
        double now = bodies.now();
        if (now >= framesTaken * Shot.FRAME_SECONDS) {
            takeAFrame();
        }
        for (Bodies.Gone gone : bodies.goneOver(Shot.LAUNCHED)) {
            History history = pieces.get(gone.name());
            if (history != null) {
                history.fate = new Fate.Gone(seconds(now), gone.side());
                history.samples.add(new Placed(seconds(now), gone.at(), gone.turn()));
            }
        }
        Optional<Colliders.Side> over = world.colliders().over(ball.at(), pollen.radiusLength());
        if (over.isPresent()) {
            return over.map(side -> new Finish.LeftTheField(ball.at(), seconds(now), side));
        }
        if (ball.restFor() >= Shot.REST_SECONDS && !bodies.anyAwakeBut(Shot.LAUNCHED)) {
            return Optional.of(new Finish.AtRest(ball.at(), seconds(now), where()));
        }
        if (Shot.tooMuchWork(bodies.work(), seconds(now))) {
            return Optional.of(new Finish.TooMuchWork(seconds(now)));
        }
        if (now >= world.longest().value()) {
            return Optional.of(new Finish.OutOfTime(seconds(now)));
        }
        return Optional.empty();
    }

    private void replay(Bodies.Span span) {
        for (Bodies.Seen woken : span.woken()) {
            sample(woken, span.now());
        }
        pressingThePollen = new ArrayList<>();
        for (Contacts.Pressing press : span.pressing()) {
            if (press.ball().name().equals(Shot.LAUNCHED)) {
                pressingThePollen.add(press);
            }
        }
        span.resolved().ifPresent(resolved -> bounced(span, resolved));
        for (Bodies.Seen slept : span.slept()) {
            sample(slept, span.now());
        }
        Bodies.Seen after = span.after();
        ball = after;
        phase = phaseOfThePollen(span.pressing(), after);
        world.colliders()
                .crossed(span.from(), after.at())
                .ifPresent(crossing -> crossings.add(new Crossed(
                        seconds(span.began() + crossing.fraction() * span.took()),
                        crossing,
                        StrictMath.sqrt(after.velocity().dot(after.velocity())))));
        if (after.at().z() > apex.pollen().at().z()) {
            apex = new Frame(seconds(span.now()), moving(after, phase));
        }
    }

    private void bounced(Bodies.Span span, Contacts.Resolved resolved) {
        Bodies.Seen before = span.before(), at = span.atResolve();
        boolean bounced = false;
        for (Contacts.Hit hit : resolved.hits()) {
            boolean mine = hit.ball().equals(Shot.LAUNCHED);
            if (!mine && !hit.against().equals(Shot.LAUNCHED)) {
                continue;
            }
            Vec3 normal = mine ? hit.normal() : hit.normal().negated();
            Vec3 touch = at.at().along(normal, -pollen.radius());
            if (!bounced) {
                frames.add(new Frame(
                        seconds(span.now()), new Moving(at.at(), before.velocity(), before.spin(), at.turn(), phase)));
            }
            impacts.add(new Impact(
                    seconds(span.now()),
                    touch,
                    normal,
                    hit,
                    before.velocity(),
                    before.spin(),
                    at.velocity(),
                    at.spin()));
            if (firstBounce.isEmpty() && hit.surface() == Surface.FLOOR) {
                Vec3 run = touch.minus(launched.at());
                firstBounce = Optional.of(new FirstBounce(
                        seconds(span.now()), touch, StrictMath.sqrt(run.x() * run.x() + run.y() * run.y())));
            }
            bounced = true;
        }
        if (bounced) {
            frames.add(new Frame(seconds(span.now()), moving(at, phaseOfThePollen(span.pressing(), at))));
        }
    }

    private void sample(Bodies.Seen body, double t) {
        History history = pieces.get(body.name());
        if (history != null) {
            history.samples.add(new Placed(seconds(t), body.at(), body.turn()));
        }
    }

    private void sampleTheAwake() {
        for (Bodies.Seen body : bodies.seen()) {
            if (body.life() == Bodies.Life.AWAKE) {
                sample(body, bodies.now());
            }
        }
    }

    private void takeAFrame() {
        frames.add(frameNow());
        sampleTheAwake();
        framesTaken = (long) StrictMath.floor(bodies.now() / Shot.FRAME_SECONDS) + 1;
    }

    private Frame frameNow() {
        return new Frame(seconds(bodies.now()), moving(ball, phase));
    }

    private Phase phaseOfThePollen(List<Contacts.Pressing> pressing, Bodies.Seen ball) {
        Optional<Contacts.Pressing> first = Optional.empty();
        for (Contacts.Pressing press : pressing) {
            if (first.isEmpty() && press.ball().name().equals(Shot.LAUNCHED)) {
                first = Optional.of(press);
            }
        }
        if (first.isEmpty()) {
            return Phase.FLYING;
        }
        double moving = StrictMath.sqrt(ball.velocity().dot(ball.velocity()))
                + pollen.radius() * StrictMath.sqrt(ball.spin().dot(ball.spin()));
        if (moving < Shot.REST_SPEED_IN_PER_S) {
            return Phase.RESTING;
        }
        double slip = first.map(press -> slipOf(press.normal(), ball)).orElse(0.0);
        return slip > Shot.REST_SPEED_IN_PER_S ? Phase.SLIDING : Phase.ROLLING;
    }

    private double slipOf(Vec3 normal, Bodies.Seen ball) {
        Vec3 contact = ball.velocity().plus(ball.spin().cross(normal.times(-pollen.radius())));
        Vec3 across = contact.minus(normal.times(contact.dot(normal)));
        return StrictMath.sqrt(across.dot(across));
    }

    private Resting where() {
        Colliders colliders = world.colliders();
        Optional<Colliders.InCell> cell = colliders.cellHolding(ball.at());
        if (cell.isPresent()) {
            return cell.<Resting>map(held -> new Resting.InCell(held.cell(), held.alliance(), held.upturned()))
                    .orElse(new Resting.OnTheFloor());
        }
        Optional<String> flower = colliders.flowerHolding(ball.at());
        if (flower.isPresent()) {
            return flower.<Resting>map(Resting.InFlower::new).orElse(new Resting.OnTheFloor());
        }
        Optional<String> on = Optional.empty();
        for (Contacts.Pressing press : pressingThePollen) {
            if (press.against() instanceof Contacts.Against.Fixed fixed && fixed.surface() == Surface.FLOOR) {
                return new Resting.OnTheFloor();
            }
            if (on.isEmpty()) {
                on = Optional.of(againstName(press.against()));
            }
        }
        return on.<Resting>map(Resting.On::new).orElse(new Resting.OnTheFloor());
    }

    private static String againstName(Contacts.Against against) {
        if (against instanceof Contacts.Against.Fixed fixed) {
            return fixed.name();
        }
        if (against instanceof Contacts.Against.Ball other) {
            return other.body().name();
        }
        return "something";
    }
}
