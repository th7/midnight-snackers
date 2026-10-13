package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.firstinspires.ftc.teamcode.simcore.Shot.Crossed;
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
    private enum Life {
        AWAKE,
        ASLEEP,
        FIXED,
        GONE
    }

    private record Kin(Vec3 at, Vec3 velocity, Vec3 spin) {}

    private static final class Body {
        private final String name;
        private final Optional<World.Piece> piece;
        private final Pollen shape;
        private Vec3 at;
        private Vec3 velocity;
        private Vec3 spin;
        private Turn turn;
        private Life life;
        private double restFor;
        private final List<Placed> samples;
        private Kin start;
        private Kin trial;
        private boolean touched;
        private final Set<Collider> touchingColliders = new HashSet<>();
        private final Set<String> touchingBodies = new HashSet<>();

        private Body(
                String name,
                Optional<World.Piece> piece,
                Pollen shape,
                Vec3 at,
                Vec3 velocity,
                Vec3 spin,
                Turn turn,
                Life life,
                double restFor,
                List<Placed> samples) {
            this.name = name;
            this.piece = piece;
            this.shape = shape;
            this.at = at;
            this.velocity = velocity;
            this.spin = spin;
            this.turn = turn;
            this.life = life;
            this.restFor = restFor;
            this.samples = new ArrayList<>(samples);
            this.start = new Kin(at, velocity, spin);
            this.trial = start;
        }

        Body copy() {
            return new Body(name, piece, shape, at, velocity, spin, turn, life, restFor, samples);
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

        Moving moving(Phase phase) {
            return new Moving(at, velocity, spin, turn, phase);
        }
    }

    private final World world;
    private final Pollen pollen;
    private final WheelPair.Launched launched;
    private final double step;
    private final List<Body> bodies;
    private final Body ball;
    private final List<Frame> frames;
    private final List<Impact> impacts;
    private final List<Crossed> crossings;
    private long steps;
    private long framesTaken;
    private double now;
    private long work;
    private Frame apex;
    private Optional<FirstBounce> firstBounce;
    private Phase phase;
    private List<Contacts.Pressing> pressingThePollen;

    private ShotRun(
            World world,
            Pollen pollen,
            WheelPair.Launched launched,
            List<Body> bodies,
            List<Frame> frames,
            List<Impact> impacts,
            List<Crossed> crossings,
            long steps,
            long framesTaken,
            double now,
            long work,
            Optional<Frame> apex,
            Optional<FirstBounce> firstBounce,
            Phase phase,
            List<Contacts.Pressing> pressingThePollen) {
        this.world = world;
        this.pollen = pollen;
        this.launched = launched;
        this.step = world.step().value();
        this.bodies = new ArrayList<>();
        for (Body body : bodies) {
            this.bodies.add(body.copy());
        }
        Body first = new Body(
                Shot.LAUNCHED,
                Optional.empty(),
                pollen,
                launched.at(),
                launched.velocity(),
                launched.spin(),
                Turn.identity(),
                Life.AWAKE,
                0,
                List.of());
        for (Body body : this.bodies) {
            if (body.name.equals(Shot.LAUNCHED)) {
                first = body;
            }
        }
        this.ball = first;
        this.frames = new ArrayList<>(frames);
        this.impacts = new ArrayList<>(impacts);
        this.crossings = new ArrayList<>(crossings);
        this.steps = steps;
        this.framesTaken = framesTaken;
        this.now = now;
        this.work = work;
        this.firstBounce = firstBounce;
        this.phase = phase;
        this.pressingThePollen = new ArrayList<>(pressingThePollen);
        this.apex = apex.orElse(new Frame(Seconds.zero(), ball.moving(phase)));
    }

    static ShotRun starting(World world, Pollen pollen, WheelPair.Launched launched) {
        List<Body> bodies = new ArrayList<>();
        bodies.add(new Body(
                Shot.LAUNCHED,
                Optional.empty(),
                pollen,
                launched.at(),
                launched.velocity(),
                launched.spin(),
                Turn.identity(),
                Life.AWAKE,
                0,
                List.of()));
        for (World.Piece piece : world.pieces()) {
            bodies.add(new Body(
                    piece.name(),
                    Optional.of(piece),
                    piece.shape(),
                    piece.at(),
                    Vec3.zero(),
                    Vec3.zero(),
                    Turn.identity(),
                    piece.fixed() ? Life.FIXED : Life.ASLEEP,
                    0,
                    List.of(new Placed(Seconds.zero(), piece.at(), Turn.identity()))));
        }
        ShotRun run = new ShotRun(
                world,
                pollen,
                launched,
                bodies,
                List.of(),
                List.of(),
                List.of(),
                0,
                0,
                0,
                0,
                Optional.empty(),
                Optional.empty(),
                Phase.FLYING,
                List.of());
        run.touchingAtTheStart();
        run.phase = run.phaseOfThePollen(run.pressings());
        run.apex = run.frameNow();
        run.takeAFrame();
        return run;
    }

    ShotRun copy() {
        return new ShotRun(
                world,
                pollen,
                launched,
                bodies,
                frames,
                impacts,
                crossings,
                steps,
                framesTaken,
                now,
                work,
                Optional.of(apex),
                firstBounce,
                phase,
                pressingThePollen);
    }

    Story story() {
        List<Track> tracks = new ArrayList<>();
        for (Body body : bodies) {
            body.piece.ifPresent(piece -> tracks.add(new Track(piece, Collections.unmodifiableList(body.samples))));
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
                work,
                seconds(now));
    }

    private static Seconds seconds(double t) {
        return Seconds.of(t).orElse(Seconds.zero());
    }

    Shot until(double until) {
        while (steps * step < until) {
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
        for (Body body : bodies) {
            if (body.life == Life.AWAKE && body.piece.isPresent()) {
                body.samples.add(new Placed(seconds(now), body.at, body.turn));
            }
        }
        return new Finished(copy().story(), finish);
    }

    private Optional<Finish> aStep() {
        int pieces = piecesOfThisStep();
        for (int i = 1; i <= pieces; i++) {
            aPiece((steps + (double) i / pieces) * step);
        }
        steps++;
        now = steps * step;
        if (now >= framesTaken * Shot.FRAME_SECONDS) {
            takeAFrame();
        }
        for (Body body : bodies) {
            if (body != ball
                    && body.life == Life.AWAKE
                    && world.colliders().over(body.at, body.reach()).isPresent()) {
                body.life = Life.GONE;
                body.samples.add(new Placed(seconds(now), body.at, body.turn));
            }
        }
        Optional<Colliders.Side> over = world.colliders().over(ball.at, ball.reach());
        if (over.isPresent()) {
            return over.map(side -> new Finish.LeftTheField(ball.at, seconds(now), side));
        }
        if (ball.restFor >= Shot.REST_SECONDS && !anyPieceAwake()) {
            return Optional.of(new Finish.AtRest(ball.at, seconds(now), where()));
        }
        if (Shot.tooMuchWork(work, seconds(now))) {
            return Optional.of(new Finish.TooMuchWork(seconds(now)));
        }
        if (now >= world.longest().value()) {
            return Optional.of(new Finish.OutOfTime(seconds(now)));
        }
        return Optional.empty();
    }

    private boolean anyPieceAwake() {
        for (Body body : bodies) {
            if (body != ball && body.life == Life.AWAKE) {
                return true;
            }
        }
        return false;
    }

    private int piecesOfThisStep() {
        double fastest = 0;
        for (Body body : bodies) {
            if (body.life == Life.AWAKE) {
                fastest = StrictMath.max(fastest, StrictMath.sqrt(body.velocity.dot(body.velocity)));
            }
        }
        double needed = StrictMath.ceil(fastest * step / Shot.MOST_TRAVEL_IN);
        return (int) StrictMath.max(1, StrictMath.min(Shot.MOST_PIECES_PER_STEP, needed));
    }

    private void takeAFrame() {
        frames.add(frameNow());
        for (Body body : bodies) {
            if (body.life == Life.AWAKE && body.piece.isPresent()) {
                body.samples.add(new Placed(seconds(now), body.at, body.turn));
            }
        }
        framesTaken = (long) StrictMath.floor(now / Shot.FRAME_SECONDS) + 1;
    }

    private Frame frameNow() {
        return new Frame(seconds(now), ball.moving(phase));
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
            body.touchingColliders.clear();
            body.touchingBodies.clear();
            if (body.life != Life.AWAKE) {
                continue;
            }
            for (Colliders.Near near : world.colliders().near(body.at, body.reach(), Shot.TOUCHING_IN)) {
                body.touchingColliders.add(near.collider());
            }
            for (Body other : bodies) {
                if (other != body
                        && other.life != Life.GONE
                        && gapBetween(body.at, body, other.at, other) <= Shot.TOUCHING_IN) {
                    body.touchingBodies.add(other.name);
                }
            }
        }
    }

    private static double gapBetween(Vec3 a, Body one, Vec3 b, Body other) {
        Vec3 apart = a.minus(b);
        return StrictMath.sqrt(apart.dot(apart)) - one.radius() - other.radius();
    }

    private void aPiece(double target) {
        int events = 0;
        while (now < target) {
            List<Body> awake = awake();
            double span = target - now;
            touchingAtTheStart();
            for (Body body : awake) {
                body.start = new Kin(body.at, body.velocity, body.spin);
            }
            Vec3 from = ball.at;
            integrate(awake, span);
            double covered = span;
            if (events < Shot.MOST_EVENTS_PER_PIECE && leastNewGap(awake) < 0) {
                double lo = 0, hi = 1;
                for (int i = 0; i < Shot.BISECTIONS; i++) {
                    double mid = (lo + hi) / 2;
                    integrate(awake, mid * span);
                    if (leastNewGap(awake) < 0) {
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
            List<Contacts.Pressing> pressing = resolve();
            project();
            settle(covered);
            phase = phaseOfThePollen(pressing);
            world.colliders()
                    .crossed(from, ball.at)
                    .ifPresent(crossing -> crossings.add(new Crossed(
                            seconds(began + crossing.fraction() * took),
                            crossing,
                            StrictMath.sqrt(ball.velocity.dot(ball.velocity)))));
            if (ball.at.z() > apex.pollen().at().z()) {
                apex = frameNow();
            }
        }
    }

    private void integrate(List<Body> awake, double over) {
        for (Body body : awake) {
            body.trial = rk4(body, body.start, over);
            work++;
        }
    }

    private Kin rk4(Body body, Kin from, double dt) {
        Air air = world.air();
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

    private double leastNewGap(List<Body> awake) {
        double least = Double.MAX_VALUE;
        for (Body body : awake) {
            for (Colliders.Near near : world.colliders().near(body.trial.at(), body.reach(), 0)) {
                if (!body.touchingColliders.contains(near.collider())) {
                    least = StrictMath.min(least, near.gap());
                }
            }
            for (Body other : bodies) {
                if (other == body || other.life == Life.GONE || body.touchingBodies.contains(other.name)) {
                    continue;
                }
                Vec3 there = other.life == Life.AWAKE ? other.trial.at() : other.at;
                least = StrictMath.min(least, gapBetween(body.trial.at(), body, there, other));
            }
        }
        return least;
    }

    private List<Contacts.Pressing> pressings() {
        List<Contacts.Pressing> pressing = new ArrayList<>();
        List<Body> awake = awake();
        for (Body body : awake) {
            body.touched = false;
        }
        for (Body body : awake) {
            for (Colliders.Near near : world.colliders().near(body.at, body.reach(), Shot.TOUCHING_IN)) {
                pressing.add(new Contacts.Pressing(
                        body.contact(),
                        new Contacts.Against.Fixed(
                                near.collider().name(), near.collider().surface()),
                        near.normal(),
                        !body.touchingColliders.contains(near.collider())));
                body.touched = true;
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
                Vec3 apart = body.at.minus(other.at);
                double distance = StrictMath.sqrt(apart.dot(apart));
                if (!(distance > 0) || distance - body.radius() - other.radius() > Shot.TOUCHING_IN) {
                    continue;
                }
                Vec3 normal = apart.times(1 / distance);
                boolean fresh = !body.touchingBodies.contains(other.name);
                body.touched = true;
                if (other.life == Life.FIXED || other.life == Life.ASLEEP) {
                    pressing.add(new Contacts.Pressing(
                            body.contact(), new Contacts.Against.Fixed(other.name, Surface.PIECE), normal, fresh));
                } else if (other == ball) {
                    other.touched = true;
                    pressing.add(new Contacts.Pressing(
                            other.contact(), new Contacts.Against.Ball(body.contact()), normal.negated(), fresh));
                } else {
                    other.touched = true;
                    pressing.add(new Contacts.Pressing(
                            body.contact(), new Contacts.Against.Ball(other.contact()), normal, fresh));
                }
            }
        }
        return pressing;
    }

    private void wake() {
        List<Body> awake = awake();
        for (Body body : awake) {
            for (Body other : bodies) {
                if (other.life == Life.ASLEEP && gapBetween(body.at, body, other.at, other) <= Shot.TOUCHING_IN) {
                    other.life = Life.AWAKE;
                    other.restFor = 0;
                    other.samples.add(new Placed(seconds(now), other.at, other.turn));
                }
            }
        }
    }

    private List<Contacts.Pressing> resolve() {
        wake();
        List<Contacts.Pressing> pressing = pressings();
        pressingThePollen = new ArrayList<>();
        for (Contacts.Pressing press : pressing) {
            if (press.ball().name().equals(Shot.LAUNCHED)) {
                pressingThePollen.add(press);
            }
        }
        if (pressing.isEmpty()) {
            return pressing;
        }
        Vec3 velocityBefore = ball.velocity, spinBefore = ball.spin;
        Contacts.Resolved resolved = Contacts.resolve(world.surfaces(), pressing);
        for (Contacts.Body moved : resolved.bodies()) {
            for (Body body : bodies) {
                if (body.name.equals(moved.name()) && body.life == Life.AWAKE) {
                    body.velocity = moved.velocity();
                    body.spin = moved.spin();
                }
            }
        }
        boolean bounced = false;
        for (Contacts.Hit hit : resolved.hits()) {
            boolean mine = hit.ball().equals(Shot.LAUNCHED);
            if (!mine && !hit.against().equals(Shot.LAUNCHED)) {
                continue;
            }
            Vec3 normal = mine ? hit.normal() : hit.normal().negated();
            Vec3 at = ball.at.along(normal, -ball.radius());
            if (!bounced) {
                frames.add(new Frame(seconds(now), new Moving(ball.at, velocityBefore, spinBefore, ball.turn, phase)));
            }
            impacts.add(
                    new Impact(seconds(now), at, normal, hit, velocityBefore, spinBefore, ball.velocity, ball.spin));
            if (firstBounce.isEmpty() && hit.surface() == Surface.FLOOR) {
                Vec3 run = at.minus(launched.at());
                firstBounce = Optional.of(
                        new FirstBounce(seconds(now), at, StrictMath.sqrt(run.x() * run.x() + run.y() * run.y())));
            }
            bounced = true;
        }
        if (bounced) {
            frames.add(new Frame(seconds(now), ball.moving(phaseOfThePollen(pressing))));
        }
        return pressing;
    }

    private void project() {
        for (int pass = 0; pass < Shot.PROJECTIONS; pass++) {
            for (Body body : awake()) {
                for (Colliders.Near near : world.colliders().near(body.at, body.reach(), 0)) {
                    if (near.gap() < 0) {
                        body.at = body.at.along(near.normal(), -near.gap());
                    }
                }
                for (Body other : bodies) {
                    if (other == body || other.life == Life.GONE) {
                        continue;
                    }
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

    private void settle(double covered) {
        for (Body body : awake()) {
            double moving = StrictMath.sqrt(body.velocity.dot(body.velocity))
                    + body.radius() * StrictMath.sqrt(body.spin.dot(body.spin));
            body.restFor = body.touched && moving < Shot.REST_SPEED_IN_PER_S ? body.restFor + covered : 0;
            if (body != ball && body.restFor >= Shot.REST_SECONDS) {
                body.life = Life.ASLEEP;
                body.velocity = Vec3.zero();
                body.spin = Vec3.zero();
                body.samples.add(new Placed(seconds(now), body.at, body.turn));
            }
        }
    }

    private Phase phaseOfThePollen(List<Contacts.Pressing> pressing) {
        Optional<Contacts.Pressing> first = Optional.empty();
        for (Contacts.Pressing press : pressing) {
            if (first.isEmpty() && press.ball().name().equals(Shot.LAUNCHED)) {
                first = Optional.of(press);
            }
        }
        if (first.isEmpty()) {
            return Phase.FLYING;
        }
        double moving = StrictMath.sqrt(ball.velocity.dot(ball.velocity))
                + ball.radius() * StrictMath.sqrt(ball.spin.dot(ball.spin));
        if (moving < Shot.REST_SPEED_IN_PER_S) {
            return Phase.RESTING;
        }
        double slip = first.map(press -> slipOf(press.normal())).orElse(0.0);
        return slip > Shot.REST_SPEED_IN_PER_S ? Phase.SLIDING : Phase.ROLLING;
    }

    private double slipOf(Vec3 normal) {
        Vec3 contact = ball.velocity.plus(ball.spin.cross(normal.times(-ball.radius())));
        Vec3 across = contact.minus(normal.times(contact.dot(normal)));
        return StrictMath.sqrt(across.dot(across));
    }

    private Resting where() {
        Colliders colliders = world.colliders();
        Optional<Colliders.InCell> cell = colliders.cellHolding(ball.at);
        if (cell.isPresent()) {
            return cell.<Resting>map(held -> new Resting.InCell(held.cell(), held.alliance(), held.upturned()))
                    .orElse(new Resting.OnTheFloor());
        }
        Optional<String> flower = colliders.flowerHolding(ball.at);
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
