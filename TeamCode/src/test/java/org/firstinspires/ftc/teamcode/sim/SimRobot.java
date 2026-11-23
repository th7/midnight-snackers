package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.dyn4j.collision.Filter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Settings;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.hull.GiftWrap;
import org.dyn4j.world.ValueMixer;
import org.dyn4j.world.World;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.reginald.opmode.OpMode;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.simcore.Chassis;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Count;
import org.firstinspires.ftc.teamcode.simcore.DeadWheels;
import org.firstinspires.ftc.teamcode.simcore.Drawn;
import org.firstinspires.ftc.teamcode.simcore.Draws;
import org.firstinspires.ftc.teamcode.simcore.Drivetrain;
import org.firstinspires.ftc.teamcode.simcore.Encoder;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Flywheels;
import org.firstinspires.ftc.teamcode.simcore.Handover;
import org.firstinspires.ftc.teamcode.simcore.Heading;
import org.firstinspires.ftc.teamcode.simcore.HubPort;
import org.firstinspires.ftc.teamcode.simcore.Intake;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.Nest;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.PerWheel;
import org.firstinspires.ftc.teamcode.simcore.Power;
import org.firstinspires.ftc.teamcode.simcore.Ramp;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Rolling;
import org.firstinspires.ftc.teamcode.simcore.RunMode;
import org.firstinspires.ftc.teamcode.simcore.Seat;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Sense;
import org.firstinspires.ftc.teamcode.simcore.Spin;
import org.firstinspires.ftc.teamcode.simcore.Spinner;
import org.firstinspires.ftc.teamcode.simcore.Stack;
import org.firstinspires.ftc.teamcode.simcore.Twist;
import org.firstinspires.ftc.teamcode.simcore.Vec2;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.firstinspires.ftc.teamcode.simcore.Wheel;
import org.firstinspires.ftc.teamcode.simcore.ZeroPower;

public class SimRobot {
    public static final Field FIELD = SimPlacement.FIELD;

    public static final double BATTERY_VOLTS = SimDevices.BATTERY_VOLTS;

    public static final double MAX_STEP_SECONDS = 0.005;

    private static final double BALL_MASS_KG = Nest.BALL_MASS_KG;

    private static final double ROLL_SECONDS = Flight.ROLL_SECONDS;

    private static final double REST_SPEED_IN_PER_S = Flight.REST_SPEED_IN_PER_S;

    private static final double BOUNCE = Flight.BOUNCE;

    private static final double BALL_FRICTION = Flight.FRICTION;

    private static final double IN = Length.METRES_PER_INCH;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    private static final double WALL_THICKNESS_M = 1;

    private final RobotDescription description;

    private final SimDevices devices;

    public final FakeDcMotorEx leftFront;
    public final FakeDcMotorEx rightFront;
    public final FakeDcMotorEx leftBack;
    public final FakeDcMotorEx rightBack;
    public final FakeDcMotorEx intake;
    public final FakeImu imu;
    public final FakeVoltageSensor voltageSensor;
    public final FakeDashboard dashboard;

    private final Noise noise;

    private Draws draws;

    private final MecanumKinematics kinematics;
    private final Drivetrain drivetrain;
    private final World<Body> world = new World<>();
    private final Body chassis;

    private Pose2d previous = new Pose2d(0, 0, 0);

    private DeadWheels deadWheels;

    private static final Map<String, Wheel> WHEELS = Map.of(
            "leftFront", Wheel.LEFT_FRONT,
            "rightFront", Wheel.RIGHT_FRONT,
            "leftBack", Wheel.LEFT_BACK,
            "rightBack", Wheel.RIGHT_BACK);

    private final Map<String, HubPort.State> ports = new LinkedHashMap<>();

    private final Map<String, Power> applied = new LinkedHashMap<>();

    private final Map<String, Count> counted = new LinkedHashMap<>();

    private final Map<String, Spin> spins = new LinkedHashMap<>();

    private static final Map<Flywheels.Side, String> FLYWHEELS =
            Map.of(Flywheels.Side.FRONT, "flywheelFront", Flywheels.Side.BACK, "flywheelBack");

    private final Flywheels flywheels;

    private final Map<Flywheels.Side, Flywheels.Turning> flywheelsTurning = new EnumMap<>(
            Map.of(Flywheels.Side.FRONT, Flywheels.Turning.still(), Flywheels.Side.BACK, Flywheels.Turning.still()));

    private final Ball[] balls;

    private final List<Body> ballBodies = new ArrayList<>();

    private final Map<Field.Flower, Stack<Ball>> stacks = new LinkedHashMap<>();

    private final Map<Field.Flower, Seat<Ball>> seats = new LinkedHashMap<>();

    private Flight<Ball> flight;

    private Intake.Held held = Intake.Held.none();

    private Intake.Way intakeWay = Intake.Way.STILL;

    private Spinner.Turning spinnerTurning = Spinner.Turning.still();

    private Ramp ramp = Ramp.stowed();

    private final Deque<Ball> holding = new ArrayDeque<>();

    private double spinnerTurns = 0;

    private int fed = 0;

    private Handover.Why handing = Handover.Why.SPINNER_NOT_CLOCKWISE;

    private enum Where {
        ROLLING,

        FLYING,

        HELD,

        FED,

        IN_FLOWER,

        OUT
    }

    public interface Piece {
        Field.Kind kind();

        double radius();

        Field.Piece setUpFrom();
    }

    private static final class Ball implements Piece {
        final double radius;

        final Field.Kind kind;

        final Field.Piece setUpFrom;

        final Body body;

        @Override
        public Field.Kind kind() {
            return kind;
        }

        @Override
        public double radius() {
            return radius;
        }

        @Override
        public Field.Piece setUpFrom() {
            return setUpFrom;
        }

        Where where;

        Field.Flower flower;

        Vec3 out;

        final Length length;

        Ball(Field.Piece setUpFrom, Body body) {
            this.setUpFrom = setUpFrom;
            this.radius = setUpFrom.radius();
            this.length = Valid.value(Length.of(radius));
            this.kind = setUpFrom.kind();
            this.body = body;
        }
    }

    private static final class GrippierDecides implements ValueMixer {
        @Override
        public double mixFriction(double one, double other) {
            return Math.max(one, other);
        }

        @Override
        public double mixRestitution(double one, double other) {
            return ValueMixer.DEFAULT_MIXER.mixRestitution(one, other);
        }

        @Override
        public double mixRestitutionVelocity(double one, double other) {
            return ValueMixer.DEFAULT_MIXER.mixRestitutionVelocity(one, other);
        }
    }

    private static final class Reaches implements Filter {
        private final double clears;
        private final double stands;

        Reaches(double clears, double stands) {
            this.clears = clears;
            this.stands = stands;
        }

        @Override
        public boolean isAllowed(Filter other) {
            if (!(other instanceof Reaches)) {
                return true;
            }
            Reaches that = (Reaches) other;
            return stands > that.clears && that.stands > clears;
        }

        @Override
        public Filter copy() {
            return this;
        }
    }

    public SimRobot() {
        this(SimNoise.NONE);
    }

    public SimRobot(Noise noise) {
        this(RobotDescription.reginalds(), noise);
    }

    public RobotDescription description() {
        return description;
    }

    public SimRobot(RobotDescription description, Noise noise) {
        this.description = description;
        this.devices = new SimDevices(description);
        this.leftFront = devices.motor("leftFront");
        this.rightFront = devices.motor("rightFront");
        this.leftBack = devices.motor("leftBack");
        this.rightBack = devices.motor("rightBack");
        this.intake = devices.motor("intake");
        this.imu = devices.imu;
        this.voltageSensor = devices.voltageSensor;
        this.dashboard = devices.dashboard;
        this.noise = noise;
        this.draws = noise.draws();
        Constants constants = description.constants();
        this.kinematics = new MecanumKinematics(
                constants.value(Constant.TRACK_WIDTH_IN), constants.value(Constant.LATERAL_MULTIPLIER));
        this.drivetrain = Valid.value(description.drivetrain(noise.motors()));
        this.deadWheels = Valid.value(description.deadWheels());
        this.flywheels = Valid.value(description.flywheels());
        for (String name : devices.motorsCommittedAs()) {
            ports.put(name, HubPort.State.fresh());
            applied.put(name, Power.none());
            counted.put(name, Count.none());
            if (!WHEELS.containsKey(name)) {
                spins.put(name, Spin.still());
            }
        }
        voltageSensor.voltage = noise.battery().volts(Seconds.zero(), PerWheel.all(Power.none()));
        Settings settings = world.getSettings();
        settings.setLinearTolerance(CONTACT_TOLERANCE_IN * IN);
        settings.setMaximumAtRestLinearVelocity(REST_SPEED_IN_PER_S * IN);
        settings.setMaximumAtRestAngularVelocity(REST_SPEED_IN_PER_S / largestBallRadius());
        settings.setMinimumAtRestTime(0.25);
        settings.setVelocityConstraintSolverIterations(20);
        settings.setPositionConstraintSolverIterations(10);
        world.setGravity(World.ZERO_GRAVITY);
        world.setValueMixer(new GrippierDecides());

        double half = SimPlacement.FIELD_SIZE_IN / 2 * IN;
        double reach = half + WALL_THICKNESS_M / 2;
        double length = 2 * half + 2 * WALL_THICKNESS_M;
        world.addBody(wall(reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(-reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(0, reach, length, WALL_THICKNESS_M));
        world.addBody(wall(0, -reach, length, WALL_THICKNESS_M));
        for (Field.Obstacle obstacle : FIELD.obstacles()) {
            world.addBody(obstacleBody(obstacle));
        }

        chassis = new Body();
        BodyFixture footprint = chassis.addFixture(
                Geometry.createRectangle(SimPlacement.ROBOT_SIZE_IN * IN, SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setDensity(constants.value(Constant.MASS_KG)
                / (SimPlacement.ROBOT_SIZE_IN * IN * SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setFriction(0);
        footprint.setRestitution(0);
        footprint.setFilter(new Reaches(0, SimPlacement.ROBOT_SIZE_IN));
        chassis.setMass(MassType.NORMAL);
        chassis.setAtRestDetectionEnabled(false);
        chassis.setLinearDamping(0);
        chassis.setAngularDamping(0);
        world.addBody(chassis);

        flight = Valid.value(Flight.<Ball>over(FIELD));
        for (Field.Flower flower : FIELD.flowers()) {
            stacks.put(flower, Stack.in(flower));
            seats.put(flower, Seat.seated());
        }
        List<Field.Piece> moved = FIELD.movedPieces();
        balls = new Ball[moved.size()];
        for (int i = 0; i < balls.length; i++) {
            Field.Piece piece = moved.get(i);
            balls[i] = new Ball(piece, ballBody(piece.radius()));
            ballBodies.add(balls[i].body);
            Field.Place place = piece.place();
            if (place instanceof Field.Place.Loose) {
                setDown(balls[i], piece.at().x(), piece.at().y());
            } else if (place instanceof Field.Place.InCell) {
                restInTheCell(balls[i], piece.at());
            } else if (place instanceof Field.Place.InFlower flowerPlace) {
                putInFlower(balls[i], flowerPlace.flower(), piece.at().z());
            }
        }
        restTheFlowers();
        standInTheBores();
    }

    private static double largestBallRadius() {
        double largest = 0;
        for (Field.Piece piece : FIELD.movedPieces()) {
            largest = Math.max(largest, piece.radius());
        }
        return largest;
    }

    private static Body wall(double x, double y, double width, double height) {
        Body wall = new Body();
        BodyFixture fixture = wall.addFixture(Geometry.createRectangle(width, height));
        fixture.setFriction(0);
        fixture.setRestitution(0);
        fixture.setFilter(new Reaches(0, SimPlacement.WALL_HEIGHT_IN));
        wall.setMass(MassType.INFINITE);
        wall.translate(x, y);
        return wall;
    }

    private static Body obstacleBody(Field.Obstacle obstacle) {
        List<Vec2> footprint = obstacle.footprint().corners();
        Vector2[] points = new Vector2[footprint.size()];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Vector2(footprint.get(i).x() * IN, footprint.get(i).y() * IN);
        }
        Vector2[] hull = new GiftWrap().generate(points);
        Body body = new Body();
        try {
            BodyFixture fixture = body.addFixture(Geometry.createPolygon(hull));
            fixture.setFriction(0);
            fixture.setRestitution(0);
            fixture.setFilter(new Reaches(obstacle.clears(), obstacle.stands()));
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(obstacle.name() + " is not a convex footprint the engine can hold", e);
        }
        body.setMass(MassType.INFINITE);
        return body;
    }

    private static Body ballBody(double radius) {
        Body body = new Body();
        BodyFixture fixture = body.addFixture(Geometry.createCircle(radius * IN));
        fixture.setDensity(BALL_MASS_KG / (Math.PI * radius * IN * radius * IN));
        fixture.setFriction(BALL_FRICTION);
        fixture.setRestitution(BOUNCE);
        fixture.setRestitutionVelocity(0);
        fixture.setFilter(new Reaches(0, 2 * radius));
        body.setMass(MassType.NORMAL);
        body.setLinearDamping(1 / ROLL_SECONDS);
        body.setAngularDamping(1 / ROLL_SECONDS);
        return body;
    }

    public void wire(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
        opMode.hardwareMap = devices.hardwareMap();
        if (opMode instanceof OpMode reginalds) {
            reginalds.useHardware(devices.hardware());
        }
    }

    public double[] powers() {
        return new double[] {leftFront.power, rightFront.power, leftBack.power, rightBack.power};
    }

    public Hardware hardware() {
        return devices.hardware();
    }

    public long nanoTime() {
        return devices.nanoTime();
    }

    public Noise noise() {
        return noise;
    }

    public double nextLoopSeconds() {
        Drawn<Seconds> loop = noise.nextLoop(draws);
        draws = loop.next();
        return loop.value().value();
    }

    public Pose2d pose() {
        Transform transform = chassis.getTransform();
        return new Pose2d(
                transform.getTranslationX() / IN, transform.getTranslationY() / IN, transform.getRotationAngle());
    }

    public void setPose(Pose2d pose) {
        Pose2d placed = SimPlacement.onTheField(pose);

        world.removeBody(chassis);
        Transform transform = chassis.getTransform();
        transform.setTranslation(placed.position.x * IN, placed.position.y * IN);
        transform.setRotation(placed.heading.toDouble());
        chassis.setLinearVelocity(new Vector2());
        chassis.setAngularVelocity(0);
        chassis.clearForce();
        chassis.clearTorque();
        world.addBody(chassis);
        previous = pose();
        imu.yawRadians = previous.heading.toDouble();
    }

    public void setDown(Pose2d pose) {
        Drawn<Noise.Nudge> nudge = noise.setDown(draws);
        draws = nudge.next();
        setPose(nudge.value()
                .fold(
                        pose,
                        by -> new Pose2d(
                                pose.position.x + by.offset().x(),
                                pose.position.y + by.offset().y(),
                                pose.heading.toDouble() + by.radians())));
    }

    public double[][] pieces() {
        double[][] out = new double[balls.length][];
        for (int i = 0; i < balls.length; i++) {
            Ball ball = balls[i];
            switch (ball.where) {
                case ROLLING:
                    Transform t = ball.body.getTransform();
                    out[i] = new double[] {t.getTranslationX() / IN, t.getTranslationY() / IN, ball.radius};
                    break;
                case FLYING:
                    out[i] = Points.array(flight.at(ball).orElseThrow());
                    break;
                case HELD:
                case FED:
                    out[i] = null;
                    break;
                case IN_FLOWER:
                    out[i] = Points.array(stacks.get(ball.flower).at(ball).orElseThrow());
                    break;
                default:
                    out[i] = Points.array(ball.out);
            }
        }
        return out;
    }

    public List<Piece> balls() {
        return List.of(balls);
    }

    public double[] placeOf(Piece piece) {
        return pieces()[indexOf(piece)];
    }

    private int indexOf(Piece piece) {
        for (int i = 0; i < balls.length; i++) {
            if (balls[i] == piece) {
                return i;
            }
        }
        throw new IllegalArgumentException("that is not one of this robot's balls");
    }

    public void place(Piece piece, double x, double y) {
        setDown((Ball) piece, x, y);
    }

    public void place(Piece piece, Field.Cell cell) {
        putInCell((Ball) piece, cell);
    }

    public void toss(Piece piece, Vec3 at, Vec3 velocity) {
        Ball ball = (Ball) piece;
        take(ball);
        ball.where = Where.FLYING;
        flight = refusedIfNot(flight.with(ball, ball.kind, ball.length, at, velocity));
    }

    private void setDown(Ball ball, double x, double y) {
        take(ball);
        ball.where = Where.ROLLING;
        ball.body.getTransform().setTranslation(x * IN, y * IN);
        ball.body.setLinearVelocity(new Vector2());
        ball.body.setAngularVelocity(0);
        ball.body.setAtRest(false);
        world.addBody(ball.body);
    }

    private void take(Ball ball) {
        if (ball.where == Where.HELD) {
            held = held.less(1);
            holding.remove(ball);
        }
        if (ball.where == Where.ROLLING) {
            world.removeBody(ball.body);
        }
        if (ball.where == Where.FLYING) {
            flight = refusedIfNot(flight.without(ball));
        }
        if (ball.where == Where.IN_FLOWER) {
            stacks.put(ball.flower, stacks.get(ball.flower).without(ball));
            ball.flower = null;
        }
    }

    private void putInCell(Ball ball, Field.Cell cell) {
        take(ball);
        ball.where = Where.FLYING;
        flight = Valid.value(flight.within(ball, ball.kind, ball.length, cell));
    }

    private void restInTheCell(Ball ball, Vec3 at) {
        take(ball);
        ball.where = Where.FLYING;
        flight = Valid.value(flight.restingAt(ball, ball.kind, ball.length, at));
    }

    private void putInFlower(Ball ball, Field.Flower flower, double z) {
        take(ball);
        ball.where = Where.IN_FLOWER;
        ball.flower = flower;
        stacks.put(flower, stacks.get(flower).with(ball, ball.length, z));
    }

    private void restTheFlowers() {
        for (Field.Flower flower : FIELD.flowers()) {
            settle(stacks.get(flower).rested(rolling()));
        }
    }

    private void fallInTheFlowers(Seconds dt) {
        for (Field.Flower flower : FIELD.flowers()) {
            settle(stacks.get(flower).after(dt, rolling()));
        }
    }

    private void standInTheBores() {
        flight = refusedIfNot(flight.amid(stacks.values(), rolling()));
    }

    private void settle(Stack.Settled<Ball> settled) {
        Field.Flower flower = settled.stack().flower();
        stacks.put(flower, settled.stack());
        for (Ball ball : settled.leaving()) {
            setDown(ball, flower.axis().x(), flower.axis().y());
        }
    }

    private List<Rolling<Ball>> rolling() {
        List<Rolling<Ball>> rolling = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.where == Where.ROLLING) {
                rolling.add(new Rolling<>(ball, inches(ball.body.getTransform()), ball.length));
            }
        }
        return rolling;
    }

    private static Vec2 inches(Transform at) {
        return new Vec2(at.getTranslationX() / IN, at.getTranslationY() / IN);
    }

    private void holdTheNests(Seconds dt) {
        List<Rolling<Ball>> rolling = rolling();
        for (Field.Flower flower : FIELD.flowers()) {
            Optional<Ball> nested = Nest.nested(flower, rolling);
            Seat.Next<Ball> next = seats.get(flower)
                    .next(
                            nested,
                            nested.map(this::pushedByTheRobot).orElse(false),
                            nested.map(ball -> ball.body.getLinearVelocity().getMagnitude())
                                    .orElse(0.0));
            seats.put(flower, next.seat());
            next.held().ifPresent(ball -> hold(flower, ball, dt));
        }
    }

    private void hold(Field.Flower flower, Ball nested, Seconds dt) {
        int standingOnIt = stacks.get(flower).size();
        Nest.hold(flower, inches(nested.body.getTransform()), nested.length, standingOnIt)
                .ifPresent(force -> nested.body.applyForce(new Vector2(force.x(), force.y())));
        Vector2 rolling = nested.body.getLinearVelocity();
        Nest.drag(standingOnIt, nested.body.getMass().getMass(), rolling.getMagnitude(), dt)
                .ifPresent(
                        drag -> nested.body.applyForce(rolling.getNormalized().multiply(-drag)));
    }

    private boolean pushedByTheRobot(Ball ball) {
        Deque<Body> touching = new ArrayDeque<>(world.getInContactBodies(chassis, false));
        List<Body> followed = new ArrayList<>();
        while (!touching.isEmpty()) {
            Body body = touching.poll();
            if (body == ball.body) {
                return true;
            }
            if (!ballBodies.contains(body) || followed.contains(body)) {
                continue;
            }
            followed.add(body);
            touching.addAll(world.getInContactBodies(body, false));
        }
        return false;
    }

    public SimRecording.Mechanisms mechanisms() {
        return new SimRecording.Mechanisms(
                held.count(),
                intakeWay,
                spinnerTurning,
                ramp.degrees(),
                ramp.up(description.constants().value(Constant.RAMP_UP_DEGREES)),
                fed,
                handing,
                flywheelsTurning.get(Flywheels.Side.FRONT),
                flywheelsTurning.get(Flywheels.Side.BACK));
    }

    public int scored(String alliance) {
        return flight.scored(alliance);
    }

    public Map<String, Integer> scored() {
        return flight.scored();
    }

    public double load(String alliance) {
        return flight.load(alliance).orElseThrow(() -> new IllegalArgumentException("no hive for " + alliance));
    }

    public double tilt(String alliance) {
        return flight.hives().tilt(hiveOf(alliance));
    }

    public Map<String, Double> tilt() {
        return flight.hives().tilts();
    }

    public Field.Cell upturnedCell(String alliance) {
        return flight.hives().upturnedCell(hiveOf(alliance)).orElseThrow();
    }

    public Field.Hive hiveOf(String alliance) {
        return flight.hives()
                .hiveOf(alliance)
                .orElseThrow(() -> new IllegalArgumentException("no hive for " + alliance));
    }

    public void step(double dtSeconds) {
        Seconds dt = Valid.value(Seconds.of(dtSeconds));
        devices.advance(dtSeconds);
        int steps = Math.max(1, (int) Math.ceil(dtSeconds / MAX_STEP_SECONDS));
        for (int i = 0; i < steps; i++) {
            substep(dt.dividedInto(steps));
        }
    }

    public double applied(String motor) {
        devices.motor(motor);
        return applied.get(motor).value();
    }

    private void substep(Seconds dt) {
        driveThePorts(dt);
        PerWheel<Drivetrain.Setting> wheels = new PerWheel<>(
                setting("leftFront", leftFront),
                setting("rightFront", rightFront),
                setting("leftBack", leftBack),
                setting("rightBack", rightBack));
        voltageSensor.voltage =
                noise.battery().volts(Valid.value(Seconds.of(nanoTime() / 1e9)), wheels.map(Drivetrain.Setting::power));

        driveTheRobot(wheels, dt);
        spinTheMotors(dt);
        holdTheNests(dt);
        world.step(1, dt.value());
        intakeThePollen();
        turnTheSpindexer(dt);
        fallInTheFlowers(dt);
        standInTheBores();
        flyTheBalls(dt);
        readTheSensors();
    }

    private void driveThePorts(Seconds dt) {
        for (String name : devices.motorsCommittedAs()) {
            HubPort.Driven driven = refusedIfNot(HubPort.drive(wrote(name), ports.get(name), counted.get(name), dt));
            ports.put(name, driven.next());
            applied.put(name, driven.power());
        }
    }

    private void spinTheMotors(Seconds dt) {
        for (Map.Entry<String, Spin> spin : spins.entrySet()) {
            String name = spin.getKey();
            if (!FLYWHEELS.containsValue(name)) {
                spin.setValue(spin.getValue().next(devices.typeOf(name), applied.get(name), voltageSensor.voltage, dt));
            }
        }
        for (Flywheels.Side side : Flywheels.Side.values()) {
            String name = FLYWHEELS.get(side);
            Spin spun = spins.get(name)
                    .next(
                            flywheels.motor(side),
                            applied.get(name),
                            voltageSensor.voltage,
                            zeroPower(devices.motor(name).getZeroPowerBehavior()),
                            dt);
            spins.put(name, spun);
            flywheelsTurning.put(
                    side,
                    flywheels.turning(
                            side,
                            spun.ticksPerSecond(),
                            devices.typeOf(name),
                            description.directions().reversed(side.mechanism())));
        }
    }

    private void intakeThePollen() {
        intakeWay = Intake.way(
                spins.get("intake").ticksPerSecond(),
                devices.typeOf("intake"),
                description.directions().reversed(Mechanism.INTAKE));
        List<Ball> touching = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.where == Where.ROLLING && ball.kind == Field.Kind.POLLEN && againstTheFront(ball)) {
                touching.add(ball);
            }
        }
        Intake.Taken taken = Intake.take(intakeWay, held, touching.size());
        for (Ball ball : touching.subList(0, taken.taken())) {
            take(ball);
            ball.where = Where.HELD;
            holding.add(ball);
        }
        held = taken.held();
    }

    private void turnTheSpindexer(Seconds dt) {
        spinnerTurning = Spinner.turning(
                devices.spinnerSignal(), description.directions().reversed(Mechanism.SPINNER));
        ramp = ramp.next(devices.rampSignal(), description.constants().value(Constant.RAMP_SECONDS_PER_60_DEGREES), dt);
        Handover.Stepped stepped = Handover.step(
                spinnerTurns,
                spinnerTurning,
                description.constants().value(Constant.SPINNER_FULL_SPEED_RPM),
                ramp.up(description.constants().value(Constant.RAMP_UP_DEGREES)),
                held.count(),
                dt);
        spinnerTurns = stepped.turns();
        handing = stepped.why();
        for (int i = 0; i < stepped.handed() && !holding.isEmpty(); i++) {
            Ball ball = holding.peek();
            take(ball);
            ball.where = Where.FED;
            fed++;
        }
    }

    private boolean againstTheFront(Ball ball) {
        Transform robotAt = chassis.getTransform();
        Transform ballAt = ball.body.getTransform();
        return Chassis.againstTheFront(
                Valid.value(Heading.ofRadians(robotAt.getRotationAngle())),
                new Vec2(
                        (ballAt.getTranslationX() - robotAt.getTranslationX()) / IN,
                        (ballAt.getTranslationY() - robotAt.getTranslationY()) / IN),
                ball.length);
    }

    private HubPort.Wrote wrote(String name) {
        FakeDcMotorEx motor = devices.motor(name);
        return new HubPort.Wrote(devices.typeOf(name), senseOf(motor), runMode(motor.getMode()), motor.written());
    }

    private static RunMode runMode(DcMotor.RunMode mode) {
        return switch (mode) {
            case RUN_WITHOUT_ENCODER -> RunMode.RUN_WITHOUT_ENCODER;
            case RUN_USING_ENCODER -> RunMode.RUN_USING_ENCODER;
            case RUN_TO_POSITION -> RunMode.RUN_TO_POSITION;
            case STOP_AND_RESET_ENCODER -> RunMode.STOP_AND_RESET_ENCODER;
            default -> throw new IllegalStateException("the SDK named a run mode the simulator does not know: " + mode);
        };
    }

    private Checked<Count> ownCount(String name) {
        Spin spin = spins.get(name);
        if (spin == null) {
            return Checked.rejected("the simulator does not know how big the drive wheels are yet, so " + name
                    + "'s encoder port cannot read its own motor: configure it to read a dead wheel or nothing");
        }
        return Checked.ok(spin.count());
    }

    private void driveTheRobot(PerWheel<Drivetrain.Setting> wheels, Seconds dt) {
        Vector2 linear = chassis.getLinearVelocity();
        Heading heading = Valid.value(Heading.ofRadians(chassis.getTransform().getRotationAngle()));
        Vec2 velocity = heading.onTheRobot(new Vec2(linear.x / IN, linear.y / IN));
        PoseVelocity2d inRobotFrame =
                new PoseVelocity2d(new Vector2d(velocity.x(), velocity.y()), chassis.getAngularVelocity());
        MecanumKinematics.WheelVelocities<Time> turning =
                kinematics.inverse(PoseVelocity2dDual.constant(inRobotFrame, 1));

        PerWheel<Double> accelerations = refusedIfNot(drivetrain.accelerations(
                wheels,
                voltageSensor.voltage,
                new PerWheel<>(
                        turning.leftFront.value(),
                        turning.rightFront.value(),
                        turning.leftBack.value(),
                        turning.rightBack.value()),
                dt,
                noise.traction()));

        Twist2d acceleration = kinematics
                .forward(new MecanumKinematics.WheelIncrements<>(
                        dual(accelerations.leftFront()),
                        dual(accelerations.leftBack()),
                        dual(accelerations.rightBack()),
                        dual(accelerations.rightFront())))
                .value();
        Vec2 pushed = heading.onTheField(new Vec2(acceleration.line.x, acceleration.line.y));
        double mass = chassis.getMass().getMass();
        chassis.applyForce(new Vector2(pushed.x() * IN * mass, pushed.y() * IN * mass));
        chassis.applyTorque(acceleration.angle * chassis.getMass().getInertia());
    }

    private static Sense senseOf(FakeDcMotorEx motor) {
        return motor.getDirection() == DcMotorSimple.Direction.REVERSE ? Sense.REVERSE : Sense.FORWARD;
    }

    private Drivetrain.Setting setting(String name, FakeDcMotorEx motor) {
        return new Drivetrain.Setting(applied.get(name), zeroPower(motor.getZeroPowerBehavior()));
    }

    private static ZeroPower zeroPower(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior == DcMotor.ZeroPowerBehavior.BRAKE) {
            return ZeroPower.BRAKE;
        }
        if (behavior == DcMotor.ZeroPowerBehavior.FLOAT) {
            return ZeroPower.FLOAT;
        }
        return ZeroPower.UNKNOWN;
    }

    private static <T> T refusedIfNot(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new IllegalStateException(rule);
        });
    }

    private static DualNum<Time> dual(double value) {
        return new DualNum<>(new double[] {value, 0});
    }

    public long flightWork() {
        return flight.work();
    }

    private void flyTheBalls(Seconds dt) {
        Flight.Stepped<Ball> stepped = refusedIfNot(flight.step(Valid.value(Flight.Tick.of(dt))));
        flight = stepped.flight();
        for (Flight.Landing<Ball> landing : stepped.landings()) {
            Ball ball = landing.ball();
            if (!(landing instanceof Flight.OnTheFloor<Ball> floor)) {
                ball.where = Where.OUT;
                ball.out = landing.at();
                continue;
            }
            ball.where = Where.ROLLING;
            world.addBody(ball.body);
            ball.body
                    .getTransform()
                    .setTranslation(floor.at().x() * IN, floor.at().y() * IN);
            ball.body.setLinearVelocity(
                    new Vector2(floor.velocity().x() * IN, floor.velocity().y() * IN));
            ball.body.setAngularVelocity(0);
            ball.body.setAtRest(false);
        }
    }

    private void readTheSensors() {
        Pose2d pose = pose();
        Twist2d delta = pose.minus(previous);
        previous = pose;
        Vector2 linear = chassis.getLinearVelocity();
        Heading heading = Valid.value(Heading.ofRadians(pose.heading.toDouble()));
        Twist perSecond =
                new Twist(heading.onTheRobot(new Vec2(linear.x / IN, linear.y / IN)), chassis.getAngularVelocity());

        Twist moved = new Twist(new Vec2(delta.line.x, delta.line.y), delta.angle);
        deadWheels = deadWheels.moved(moved);
        DeadWheels.Counts rolled = deadWheels.counts(perSecond);
        for (String name : devices.motorsCommittedAs()) {
            Count raw = refusedIfNot(devices.encoderOf(name).count(ownCount(name), rolled));
            counted.put(name, raw);
            Encoder reported = HubPort.reports(wrote(name), ports.get(name), raw);
            FakeDcMotorEx motor = devices.motor(name);
            motor.currentPosition = reported.position();
            motor.measuredVelocity = reported.velocity();
        }

        imu.yawRadians = pose.heading.toDouble();
        imu.yawRateRadiansPerSecond = perSecond.angle();
    }
}
