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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
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
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class SimRobot {
    public static final SimField FIELD = SimPlacement.FIELD;

    public static final double BATTERY_VOLTS = SimDevices.BATTERY_VOLTS;

    public static final int PRELOAD = 4;

    public static final int HOLDS = 4;

    public static final double MAX_STEP_SECONDS = 0.005;

    public static final double LAUNCH_HEIGHT_IN = 14;

    public static final double LAUNCH_AHEAD_IN = 6;

    public static final double LAUNCH_ANGLE_RADIANS = Math.toRadians(70);

    public static final double LAUNCH_IN_PER_S_PER_TICK_PER_S = 0.189;

    private static final double ROBOT_MASS_KG = 15;

    private static final double BALL_MASS_KG = 0.5;

    private static final double INTAKE_REACH_IN = 0.25;

    private static final double TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER = 1700;

    private static final double ROLL_SECONDS = SimFlight.ROLL_SECONDS;

    private static final double REST_SPEED_IN_PER_S = SimFlight.REST_SPEED_IN_PER_S;

    private static final double BOUNCE = SimFlight.BOUNCE;

    private static final double BALL_FRICTION = SimFlight.FRICTION;

    private static final double GRAVITY_IN_PER_S2 = SimFlight.GRAVITY_IN_PER_S2;

    private static final double NEST_FRICTION = 0.5;

    private static final double LANDING_SPEED_IN_PER_S = SimFlight.LANDING_SPEED_IN_PER_S;

    private static final double TOP_GATE_OPENS_AT = 0.8;

    private static final double BOTTOM_GATE_OPENS_AT = 0.45;

    private static final double IN = 0.0254;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    private static final double WALL_THICKNESS_M = 1;

    private static final int PAR_RAW_SIGN = 1;

    private static final int PERP_RAW_SIGN = -1;

    private static final double HUB_VELOCITY_STEP_TICKS_PER_S = 20;

    private static final int LEFT_FRONT_MOUNT = 1;

    private static final int RIGHT_FRONT_MOUNT = 1;
    private static final int LEFT_BACK_MOUNT = -1;
    private static final int RIGHT_BACK_MOUNT = -1;

    private final SimDevices devices = new SimDevices();

    public final FakeDcMotorEx leftFront = devices.leftFront;
    public final FakeDcMotorEx rightFront = devices.rightFront;
    public final FakeDcMotorEx leftBack = devices.leftBack;
    public final FakeDcMotorEx rightBack = devices.rightBack;
    public final FakeDcMotorEx launcher = devices.launcher;
    public final FakeDcMotorEx turnTable = devices.turnTable;
    public final FakeDcMotorEx intake = devices.intake;
    public final FakeServo topGate = devices.topGate;
    public final FakeServo bottomGate = devices.bottomGate;
    public final FakeImu imu = devices.imu;
    public final FakeVoltageSensor voltageSensor = devices.voltageSensor;
    public final FakeDashboard dashboard = devices.dashboard;

    private final SimNoise noise;

    private final MecanumDrive.Params drive = MecanumDrive.PARAMS;
    private final TwoDeadWheelLocalizer.Params deadWheels = TwoDeadWheelLocalizer.PARAMS;
    private final MecanumKinematics kinematics =
            new MecanumKinematics(drive.inPerTick * drive.trackWidthTicks, drive.inPerTick / drive.lateralInPerTick);
    private final World<Body> world = new World<>();
    private final Body robot;

    private Pose2d previous = new Pose2d(0, 0, 0);

    private double parTicks = 0;
    private double perpTicks = 0;

    private final Ball[] balls;

    private final List<Body> ballBodies = new ArrayList<>();

    private final Deque<Ball> hopper = new ArrayDeque<>();

    private Ball chambered = null;

    private final Map<SimField.Flower, List<Ball>> inFlower = new LinkedHashMap<>();

    private final Map<SimField.Flower, Ball> offTheSeat = new LinkedHashMap<>();

    private final SimHives hives;

    private final SimFlight flight;

    private enum Where {
        ROLLING,

        FLYING,

        HELD,

        IN_FLOWER,

        OUT
    }

    public interface Piece {
        String kind();

        double radius();

        Optional<SimField.Piece> setUpFrom();
    }

    private static final class Ball implements Piece {
        final double radius;

        final String kind;

        SimField.Piece setUpFrom;

        final Body body;

        @Override
        public String kind() {
            return kind;
        }

        @Override
        public double radius() {
            return radius;
        }

        @Override
        public Optional<SimField.Piece> setUpFrom() {
            return Optional.ofNullable(setUpFrom);
        }

        Where where;

        SimField.Flower flower;

        double x, y, z, vx, vy, vz;

        Ball(double radius, String kind, Body body) {
            this.radius = radius;
            this.kind = kind;
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

    public SimRobot(SimNoise noise) {
        this.noise = noise;
        voltageSensor.voltage = noise.batteryVolts(0, 0);
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
        for (SimField.Obstacle obstacle : FIELD.obstacles) {
            world.addBody(obstacleBody(obstacle));
        }

        robot = new Body();
        BodyFixture footprint = robot.addFixture(
                Geometry.createRectangle(SimPlacement.ROBOT_SIZE_IN * IN, SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setDensity(ROBOT_MASS_KG / (SimPlacement.ROBOT_SIZE_IN * IN * SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setFriction(0);
        footprint.setRestitution(0);
        footprint.setFilter(new Reaches(0, SimPlacement.ROBOT_SIZE_IN));
        robot.setMass(MassType.NORMAL);
        robot.setAtRestDetectionEnabled(false);
        robot.setLinearDamping(0);
        robot.setAngularDamping(0);
        world.addBody(robot);

        hives = new SimHives(FIELD);
        flight = new SimFlight(FIELD, hives);
        for (SimField.Flower flower : FIELD.flowers) {
            inFlower.put(flower, new ArrayList<>());
        }
        int loose = FIELD.loosePieces.size();
        int inCells = loose + FIELD.cellPieces.size();
        int held = inCells + FIELD.flowerPieces.size();
        balls = new Ball[held + PRELOAD];
        for (int i = 0; i < balls.length; i++) {
            SimField.Piece piece = i < held ? FIELD.movedPieces.get(i) : FIELD.loosePieces.get(0);
            balls[i] = new Ball(piece.radius, piece.kind, ballBody(piece.radius));
            balls[i].setUpFrom = i < held ? piece : null;
            ballBodies.add(balls[i].body);
            if (i < loose) {
                setDown(balls[i], piece.x, piece.y);
            } else if (i < inCells) {
                intoTheAir(balls[i], new double[] {piece.x, piece.y, piece.z}, new double[3]);
            } else if (i < held) {
                putInFlower(balls[i], FIELD.flower(piece.flower), piece.z);
            } else {
                intoTheHopper(balls[i]);
            }
        }
        restTheFlowers();
    }

    private static double largestBallRadius() {
        double largest = 0;
        for (SimField.Piece piece : FIELD.movedPieces) {
            largest = Math.max(largest, piece.radius);
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

    private static Body obstacleBody(SimField.Obstacle obstacle) {
        Vector2[] points = new Vector2[obstacle.footprint.length];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Vector2(obstacle.footprint[i][0] * IN, obstacle.footprint[i][1] * IN);
        }
        Vector2[] hull = new GiftWrap().generate(points);
        Body body = new Body();
        try {
            BodyFixture fixture = body.addFixture(Geometry.createPolygon(hull));
            fixture.setFriction(0);
            fixture.setRestitution(0);
            fixture.setFilter(new Reaches(obstacle.clears, obstacle.stands));
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(obstacle.name + " is not a convex footprint the engine can hold", e);
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

    public Hardware hardware() {
        return devices.hardware();
    }

    public Hardware hardware(Supplier<List<AprilTagDetection>> aprilTags) {
        return devices.hardware(aprilTags);
    }

    public long nanoTime() {
        return devices.nanoTime();
    }

    public SimNoise noise() {
        return noise;
    }

    public Pose2d pose() {
        Transform transform = robot.getTransform();
        return new Pose2d(
                transform.getTranslationX() / IN, transform.getTranslationY() / IN, transform.getRotationAngle());
    }

    public void setPose(Pose2d pose) {
        Pose2d placed = SimPlacement.onTheField(pose);

        world.removeBody(robot);
        Transform transform = robot.getTransform();
        transform.setTranslation(placed.position.x * IN, placed.position.y * IN);
        transform.setRotation(placed.heading.toDouble());
        robot.setLinearVelocity(new Vector2());
        robot.setAngularVelocity(0);
        robot.clearForce();
        robot.clearTorque();
        world.addBody(robot);
        previous = pose();
        imu.yawRadians = previous.heading.toDouble();
    }

    public void setDown(Pose2d pose) {
        setPose(noise.placed(pose));
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
                case HELD:
                    out[i] = null;
                    break;
                case FLYING:
                    out[i] = flight.at(ball);
                    break;
                default:
                    out[i] = new double[] {ball.x, ball.y, ball.z};
            }
        }
        return out;
    }

    public List<Piece> balls() {
        return List.of(balls);
    }

    public List<Piece> holding() {
        List<Piece> out = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.where == Where.HELD) {
                out.add(ball);
            }
        }
        return out;
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

    public void place(Piece piece, SimField.Cell cell) {
        putInCell((Ball) piece, cell);
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
            hopper.remove(ball);
            if (chambered == ball) {
                chambered = null;
            }
        }
        if (ball.where == Where.ROLLING) {
            world.removeBody(ball.body);
        }
        if (ball.where == Where.FLYING) {
            flight.remove(ball);
        }
        if (ball.where == Where.IN_FLOWER) {
            inFlower.get(ball.flower).remove(ball);
            ball.flower = null;
        }
    }

    private void putInCell(Ball ball, SimField.Cell cell) {
        take(ball);
        ball.where = Where.FLYING;
        flight.putIn(ball, ball.kind, ball.radius, cell);
    }

    private void intoTheAir(Ball ball, double[] at, double[] velocity) {
        take(ball);
        ball.where = Where.FLYING;
        flight.add(ball, ball.kind, ball.radius, at, velocity);
    }

    private void putInFlower(Ball ball, SimField.Flower flower, double z) {
        take(ball);
        ball.where = Where.IN_FLOWER;
        ball.flower = flower;
        ball.x = flower.axis[0];
        ball.y = flower.axis[1];
        ball.z = z;
        ball.vx = ball.vy = ball.vz = 0;
        List<Ball> stack = inFlower.get(flower);
        int place = 0;
        while (place < stack.size() && stack.get(place).z < z) {
            place++;
        }
        stack.add(place, ball);
    }

    private void restTheFlowers() {
        for (SimField.Flower flower : FIELD.flowers) {
            double under = topOfTheBore(flower);
            for (Ball ball : new ArrayList<>(inFlower.get(flower))) {
                ball.z = under + ball.radius;
                ball.vz = 0;
                under = settled(ball, flower);
            }
        }
    }

    private void fallInTheFlowers(double dt) {
        for (SimField.Flower flower : FIELD.flowers) {
            double under = topOfTheBore(flower);
            for (Ball ball : new ArrayList<>(inFlower.get(flower))) {
                double resting = under + ball.radius;
                if (ball.z > resting) {
                    ball.vz -= GRAVITY_IN_PER_S2 * dt;
                    ball.z += ball.vz * dt;
                }
                if (ball.z <= resting) {
                    ball.z = resting;
                    ball.vz = -ball.vz < LANDING_SPEED_IN_PER_S ? 0 : -ball.vz * BOUNCE;
                }
                under = settled(ball, flower);
            }
        }
    }

    private double settled(Ball ball, SimField.Flower flower) {
        if (ball.vz == 0 && ball.z + ball.radius <= flower.lip) {
            setDown(ball, flower.axis[0], flower.axis[1]);
        }
        return ball.z + ball.radius;
    }

    private void holdTheNests(double dt) {
        for (SimField.Flower flower : FIELD.flowers) {
            Ball nested = nestedIn(flower);
            if (nested == null) {
                offTheSeat.remove(flower);
                continue;
            }
            if (pushedByTheRobot(nested)) {
                offTheSeat.put(flower, nested);
            }
            if (offTheSeat.get(flower) != nested) {
                offTheSeat.remove(flower);
            } else if (nested.body.getLinearVelocity().getMagnitude() > REST_SPEED_IN_PER_S * IN) {
                continue;
            } else {
                offTheSeat.remove(flower);
            }
            Transform at = nested.body.getTransform();
            double toTheAxis = flower.axis[0] - at.getTranslationX() / IN;
            double acrossToIt = flower.axis[1] - at.getTranslationY() / IN;
            double out = Math.hypot(toTheAxis, acrossToIt);
            if (out > CONTACT_TOLERANCE_IN) {
                double hold = overTheRing(flower, nested) * Math.min(1, out / flower.bore);
                nested.body.applyForce(new Vector2(hold * toTheAxis / out, hold * acrossToIt / out));
            }
            Vector2 rolling = nested.body.getLinearVelocity();
            double speed = rolling.getMagnitude();
            if (speed > 0) {
                double drag = Math.min(
                        NEST_FRICTION * loadOn(flower), nested.body.getMass().getMass() * speed / dt);
                nested.body.applyForce(rolling.getNormalized().multiply(-drag));
            }
        }
    }

    private Ball nestedIn(SimField.Flower flower) {
        Ball nested = null;
        double nearest = Double.MAX_VALUE;
        for (Ball ball : balls) {
            if (ball.where != Where.ROLLING) {
                continue;
            }
            Transform at = ball.body.getTransform();
            double x = at.getTranslationX() / IN, y = at.getTranslationY() / IN;
            if (!flower.standsIn(x, y)) {
                continue;
            }
            double out = Math.hypot(x - flower.axis[0], y - flower.axis[1]);
            if (out < nearest) {
                nearest = out;
                nested = ball;
            }
        }
        return nested;
    }

    private boolean pushedByTheRobot(Ball ball) {
        Deque<Body> touching = new ArrayDeque<>(world.getInContactBodies(robot, false));
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

    private double loadOn(SimField.Flower flower) {
        return (1 + inFlower.get(flower).size()) * BALL_MASS_KG * GRAVITY_IN_PER_S2 * IN;
    }

    private double overTheRing(SimField.Flower flower, Ball ball) {
        return loadOn(flower)
                * Math.sqrt(2 * ball.radius * flower.nest - flower.nest * flower.nest)
                / (ball.radius - flower.nest);
    }

    private double topOfTheBore(SimField.Flower flower) {
        double top = 0;
        for (Ball ball : balls) {
            if (ball.where != Where.ROLLING) {
                continue;
            }
            Transform at = ball.body.getTransform();
            if (flower.standsIn(at.getTranslationX() / IN, at.getTranslationY() / IN)) {
                top = Math.max(top, 2 * ball.radius);
            }
        }
        return top;
    }

    public int held() {
        return hopper.size() + (chambered == null ? 0 : 1);
    }

    public int scored(String alliance) {
        return flight.scored(alliance);
    }

    public Map<String, Integer> scored() {
        return flight.scored();
    }

    public double load(String alliance) {
        return flight.load(alliance);
    }

    public double tilt(String alliance) {
        return hives.tilt(alliance);
    }

    public Map<String, Double> tilt() {
        return hives.tilts();
    }

    public SimField.Cell upturnedCell(String alliance) {
        return hives.upturnedCell(alliance);
    }

    public SimField.Hive hiveOf(String alliance) {
        return hives.hiveOf(alliance);
    }

    public void step(double dtSeconds) {
        devices.advance(dtSeconds);
        int steps = Math.max(1, (int) Math.ceil(dtSeconds / MAX_STEP_SECONDS));
        for (int i = 0; i < steps; i++) {
            substep(dtSeconds / steps);
        }
    }

    private void substep(double dt) {
        launcher.measuredVelocity = launcher.commandedVelocity;
        turnTable.currentPosition +=
                (int) Math.round(clamp(turnTable.power) * TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER * dt);

        double drivePower = Math.abs(clamp(leftFront.power))
                + Math.abs(clamp(rightFront.power))
                + Math.abs(clamp(leftBack.power))
                + Math.abs(clamp(rightBack.power));
        voltageSensor.voltage = noise.batteryVolts(nanoTime() / 1e9, drivePower);

        driveTheRobot(dt);
        feedTheLauncher();
        holdTheNests(dt);
        world.step(1, dt);
        intakeTheBalls();
        fallInTheFlowers(dt);
        flyTheBalls(dt);
        readTheSensors(dt);
    }

    private void driveTheRobot(double dt) {
        Vector2 linear = robot.getLinearVelocity();
        double heading = robot.getTransform().getRotationAngle();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vx = linear.x / IN, vy = linear.y / IN;
        PoseVelocity2d inRobotFrame =
                new PoseVelocity2d(new Vector2d(cos * vx + sin * vy, -sin * vx + cos * vy), robot.getAngularVelocity());
        MecanumKinematics.WheelVelocities<Time> wheels =
                kinematics.inverse(PoseVelocity2dDual.constant(inRobotFrame, 1));

        double lf = wheelAcceleration(
                leftFront, LEFT_FRONT_MOUNT, noise.motor(SimNoise.LEFT_FRONT), wheels.leftFront.value(), dt);
        double lb = wheelAcceleration(
                leftBack, LEFT_BACK_MOUNT, noise.motor(SimNoise.LEFT_BACK), wheels.leftBack.value(), dt);
        double rb = wheelAcceleration(
                rightBack, RIGHT_BACK_MOUNT, noise.motor(SimNoise.RIGHT_BACK), wheels.rightBack.value(), dt);
        double rf = wheelAcceleration(
                rightFront, RIGHT_FRONT_MOUNT, noise.motor(SimNoise.RIGHT_FRONT), wheels.rightFront.value(), dt);

        Twist2d acceleration = kinematics
                .forward(new MecanumKinematics.WheelIncrements<>(dual(lf), dual(lb), dual(rb), dual(rf)))
                .value();
        double ax = cos * acceleration.line.x - sin * acceleration.line.y;
        double ay = sin * acceleration.line.x + cos * acceleration.line.y;
        double mass = robot.getMass().getMass();
        robot.applyForce(new Vector2(ax * IN * mass, ay * IN * mass));
        robot.applyTorque(acceleration.angle * robot.getMass().getInertia());
    }

    private double wheelAcceleration(
            FakeDcMotorEx motor, int mount, SimNoise.Motor factors, double velocity, double dt) {
        int direction = motor.getDirection() == DcMotorSimple.Direction.REVERSE ? -1 : 1;
        double volts = mount * direction * clamp(motor.power) * voltageSensor.voltage;
        double kSVolts = drive.kSVolts * factors.kSVolts,
                kVVoltSecondsPerTick = drive.kVVoltSecondsPerTick * factors.kVVoltSecondsPerTick,
                kAVoltSecondsSquaredPerTick = drive.kAVoltSecondsSquaredPerTick * factors.kAVoltSecondsSquaredPerTick;
        double ticksPerSecond = velocity / drive.inPerTick;
        boolean creeping = Math.abs(ticksPerSecond) <= kSVolts / kAVoltSecondsSquaredPerTick * dt;
        double acceleration;
        if (creeping && Math.abs(volts) <= kSVolts) {
            acceleration = -velocity / dt;
        } else {
            double sign = ticksPerSecond != 0 ? Math.signum(ticksPerSecond) : Math.signum(volts);

            double backEmf =
                    clamp(motor.power) == 0 && !brakesAtZeroPower(motor) ? 0 : kVVoltSecondsPerTick * ticksPerSecond;
            acceleration = (volts - kSVolts * sign - backEmf) / kAVoltSecondsSquaredPerTick * drive.inPerTick;
        }
        double traction = noise.tractionInPerS2;
        return Math.max(-traction, Math.min(traction, acceleration));
    }

    private static boolean brakesAtZeroPower(FakeDcMotorEx motor) {
        DcMotor.ZeroPowerBehavior behavior = motor.getZeroPowerBehavior();
        if (behavior == DcMotor.ZeroPowerBehavior.BRAKE) {
            return true;
        }
        if (behavior == DcMotor.ZeroPowerBehavior.FLOAT) {
            return false;
        }
        throw new IllegalStateException("a wheel is rolling at zero power with its zero power behavior "
                + behavior + ": set BRAKE or FLOAT on it, as MecanumDrive does, so this simulation knows"
                + " whether its motor holds it back or lets it roll");
    }

    private static DualNum<Time> dual(double value) {
        return new DualNum<>(new double[] {value, 0});
    }

    private void intakeTheBalls() {
        if (clamp(intake.power) <= 0) {
            return;
        }
        for (Ball ball : balls) {
            if (held() >= HOLDS) {
                return;
            }
            if (ball.where == Where.ROLLING && SimField.POLLEN.equals(ball.kind) && againstTheFront(ball)) {
                intoTheHopper(ball);
            }
        }
    }

    private boolean againstTheFront(Ball ball) {
        Transform robotAt = robot.getTransform();
        Transform ballAt = ball.body.getTransform();
        double heading = robotAt.getRotationAngle();
        double dx = (ballAt.getTranslationX() - robotAt.getTranslationX()) / IN;
        double dy = (ballAt.getTranslationY() - robotAt.getTranslationY()) / IN;
        double ahead = Math.cos(heading) * dx + Math.sin(heading) * dy;
        double across = -Math.sin(heading) * dx + Math.cos(heading) * dy;
        double half = SimPlacement.ROBOT_SIZE_IN / 2;
        return ahead > 0 && ahead - ball.radius <= half + INTAKE_REACH_IN && Math.abs(across) <= half;
    }

    private void intoTheHopper(Ball ball) {
        take(ball);
        ball.where = Where.HELD;
        ball.vx = ball.vy = ball.vz = 0;
        hopper.add(ball);
    }

    private void feedTheLauncher() {
        if (topGate.position >= TOP_GATE_OPENS_AT && chambered == null && !hopper.isEmpty()) {
            chambered = hopper.poll();
        }
        if (bottomGate.position >= BOTTOM_GATE_OPENS_AT && chambered != null) {
            launch(chambered);
            chambered = null;
        }
    }

    private void launch(Ball ball) {
        Pose2d pose = pose();
        double aim = pose.heading.toDouble() + turnTableOffsetRadians();
        double speed = Math.abs(launcher.measuredVelocity) * LAUNCH_IN_PER_S_PER_TICK_PER_S;
        Vector2 robotVelocity = robot.getLinearVelocity();
        intoTheAir(
                ball,
                new double[] {
                    pose.position.x + LAUNCH_AHEAD_IN * Math.cos(aim),
                    pose.position.y + LAUNCH_AHEAD_IN * Math.sin(aim),
                    LAUNCH_HEIGHT_IN
                },
                new double[] {
                    robotVelocity.x / IN + speed * Math.cos(LAUNCH_ANGLE_RADIANS) * Math.cos(aim),
                    robotVelocity.y / IN + speed * Math.cos(LAUNCH_ANGLE_RADIANS) * Math.sin(aim),
                    speed * Math.sin(LAUNCH_ANGLE_RADIANS)
                });
    }

    private double turnTableOffsetRadians() {
        return (double) turnTable.currentPosition / Turntable.TICKS_PER_REVOLUTION * 2 * Math.PI;
    }

    private void flyTheBalls(double dt) {
        for (SimFlight.Landing landing : flight.step(dt)) {
            Ball ball = (Ball) landing.ball;
            if (landing.out) {
                ball.where = Where.OUT;
                ball.x = landing.at[0];
                ball.y = landing.at[1];
                ball.z = landing.at[2];
                continue;
            }
            ball.where = Where.ROLLING;
            world.addBody(ball.body);
            ball.body.getTransform().setTranslation(landing.at[0] * IN, landing.at[1] * IN);
            ball.body.setLinearVelocity(new Vector2(landing.velocity[0] * IN, landing.velocity[1] * IN));
            ball.body.setAngularVelocity(0);
            ball.body.setAtRest(false);
        }
    }

    private void readTheSensors(double dt) {
        Pose2d pose = pose();
        Twist2d delta = pose.minus(previous);
        previous = pose;
        Vector2 linear = robot.getLinearVelocity();
        double heading = pose.heading.toDouble();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vx = linear.x / IN, vy = linear.y / IN;
        PoseVelocity2d velocity =
                new PoseVelocity2d(new Vector2d(cos * vx + sin * vy, -sin * vx + cos * vy), robot.getAngularVelocity());

        parTicks += delta.line.x / drive.inPerTick + deadWheels.parYTicks * delta.angle;
        perpTicks += delta.line.y / drive.inPerTick + deadWheels.perpXTicks * delta.angle;
        double parVelocity = velocity.linearVel.x / drive.inPerTick + deadWheels.parYTicks * velocity.angVel;
        double perpVelocity = velocity.linearVel.y / drive.inPerTick + deadWheels.perpXTicks * velocity.angVel;
        rightBack.currentPosition = (int) Math.round(PAR_RAW_SIGN * parTicks);
        rightBack.measuredVelocity = asTheHubReports(PAR_RAW_SIGN * parVelocity);
        leftFront.currentPosition = (int) Math.round(PERP_RAW_SIGN * perpTicks);
        leftFront.measuredVelocity = asTheHubReports(PERP_RAW_SIGN * perpVelocity);

        imu.yawRadians = heading;
        imu.yawRateRadiansPerSecond = velocity.angVel;
    }

    private static double asTheHubReports(double ticksPerSecond) {
        return Math.round(ticksPerSecond / HUB_VELOCITY_STEP_TICKS_PER_S) * HUB_VELOCITY_STEP_TICKS_PER_S;
    }

    private static double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }
}
