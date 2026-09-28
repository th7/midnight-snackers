package org.firstinspires.ftc.teamcode.sim;

import static org.firstinspires.ftc.teamcode.sim.SimHives.along;
import static org.firstinspires.ftc.teamcode.sim.SimHives.cross;
import static org.firstinspires.ftc.teamcode.sim.SimHives.dot;
import static org.firstinspires.ftc.teamcode.sim.SimHives.length;
import static org.firstinspires.ftc.teamcode.sim.SimHives.scale;
import static org.firstinspires.ftc.teamcode.sim.SimHives.sub;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Field;

public final class SimFlight {
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

    public static final class Landing {
        public final Object ball;

        public final boolean out;

        public final double[] at;

        public final double[] velocity;

        Landing(Object ball, boolean out, double[] at, double[] velocity) {
            this.ball = ball;
            this.out = out;
            this.at = at;
            this.velocity = velocity;
        }
    }

    private static final class Ball {
        final Object key;
        final Field.Kind kind;
        final double radius;
        final double[] at;
        final double[] velocity;
        double[] spin = new double[3];
        double slowFor;
        boolean awake = true;
        boolean resting;

        Ball(Object key, Field.Kind kind, double radius, double[] at, double[] velocity) {
            this.key = key;
            this.kind = kind;
            this.radius = radius;
            this.at = at.clone();
            this.velocity = velocity.clone();
        }

        double inverseMass() {
            return awake ? 1 : 0;
        }
    }

    private static final class Contact {
        final Ball a;
        final Ball b;
        final double[] normal;
        final double[] surface;
        final double bounceTo;
        double pushed;
        double[] rubbed = new double[3];

        Contact(Ball a, Ball b, double[] normal, double[] surface) {
            this.a = a;
            this.b = b;
            this.normal = normal;
            this.surface = surface;
            double closing = dot(relative(), normal);
            this.bounceTo = -closing > LANDING_SPEED_IN_PER_S ? -BOUNCE * closing : 0;
        }

        double[] relative() {
            double[] mine = along(a.velocity, cross(a.spin, normal), -a.radius);
            double[] theirs = b == null ? surface : along(b.velocity, cross(b.spin, normal), b.radius);
            return sub(mine, theirs);
        }

        void solve() {
            double ia = a.inverseMass(), ib = b == null ? 0 : b.inverseMass();
            if (ia + ib == 0) {
                return;
            }
            double push = (bounceTo - dot(relative(), normal)) / (ia + ib);
            double total = Math.max(0, pushed + push);
            push = total - pushed;
            pushed = total;
            shove(scale(normal, push), ia, ib);

            double[] slip = relative();
            slip = along(slip, normal, -dot(slip, normal));
            double[] rub = scale(slip, -1 / ((ia + ib) * (1 + 1 / SPIN_INERTIA)));
            double[] rubbing = along(rubbed, rub, 1);
            double most = FRICTION * pushed;
            if (length(rubbing) > most) {
                rubbing = scale(rubbing, most / length(rubbing));
            }
            rub = sub(rubbing, rubbed);
            rubbed = rubbing;
            shove(rub, ia, ib);
            a.spin = along(a.spin, cross(normal, rub), -ia / (SPIN_INERTIA * a.radius));
            if (b != null) {
                b.spin = along(b.spin, cross(normal, rub), -ib / (SPIN_INERTIA * b.radius));
            }
        }

        private void shove(double[] impulse, double ia, double ib) {
            for (int axis = 0; axis < 3; axis++) {
                a.velocity[axis] += impulse[axis] * ia;
                if (b != null) {
                    b.velocity[axis] -= impulse[axis] * ib;
                }
            }
        }
    }

    private final Field field;
    private final SimHives hives;
    private final List<Ball> balls = new ArrayList<>();
    private final Map<Object, Ball> byKey = new IdentityHashMap<>();

    public SimFlight(Field field, SimHives hives) {
        this.field = field;
        this.hives = hives;
    }

    public void add(Object ball, Field.Kind kind, double radius, double[] at, double[] velocity) {
        remove(ball);
        Ball flying = new Ball(ball, kind, radius, at, velocity);
        balls.add(flying);
        byKey.put(ball, flying);
    }

    public void putIn(Object ball, Field.Kind kind, double radius, Field.Cell cell) {
        remove(ball);
        for (double[] spot : hives.restingSpots(cell, radius)) {
            if (clear(spot, radius)) {
                add(ball, kind, radius, spot, new double[3]);
                return;
            }
        }
        throw new IllegalStateException("there is no room left in " + cell.name() + " for another " + kind.named());
    }

    private boolean clear(double[] spot, double radius) {
        for (Ball other : balls) {
            if (length(sub(other.at, spot)) < other.radius + radius - CONTACT_TOLERANCE_IN) {
                return false;
            }
        }
        return true;
    }

    public void remove(Object ball) {
        Ball was = byKey.remove(ball);
        if (was != null) {
            balls.remove(was);
        }
    }

    public boolean holds(Object ball) {
        return byKey.containsKey(ball);
    }

    public double[] at(Object ball) {
        Ball flying = byKey.get(ball);
        if (flying == null) {
            throw new IllegalArgumentException("that ball is not in the air");
        }
        return flying.at.clone();
    }

    public int fill(Field.Hive hive) {
        Field.Cell up = hives.upturnedCell(hive.alliance());
        int fill = 0;
        for (Ball ball : balls) {
            if (hives.cellHolding(ball.at) == up) {
                fill += SimHives.fills(ball.kind);
            }
        }
        return fill;
    }

    public double load(String alliance) {
        return fill(hives.hiveOf(alliance)) / (double) SimHives.FULL;
    }

    public int scored(String alliance) {
        return scored().getOrDefault(alliance, 0);
    }

    public Map<String, Integer> scored() {
        Map<String, Integer> out = new LinkedHashMap<>();
        for (Field.Cell cell : field.cells()) {
            out.putIfAbsent(cell.alliance(), 0);
        }
        for (Ball ball : balls) {
            Field.Cell cell = hives.cellHolding(ball.at);
            if (cell != null) {
                out.merge(cell.alliance(), 1, Integer::sum);
            }
        }
        return out;
    }

    public List<Landing> step(double seconds) {
        double fastest = 0;
        for (Ball ball : balls) {
            fastest = Math.max(fastest, (length(ball.velocity) + GRAVITY_IN_PER_S2 * seconds) / ball.radius);
        }
        int steps = (int) Math.min(MOST_STEPS, Math.max(1, Math.ceil(seconds * fastest / RADII_PER_STEP)));
        List<Landing> landed = new ArrayList<>();
        for (int i = 0; i < steps; i++) {
            substep(seconds / steps, landed);
        }
        return landed;
    }

    private void substep(double dt, List<Landing> landed) {
        wake();
        for (Ball ball : balls) {
            if (ball.awake) {
                ball.velocity[2] -= GRAVITY_IN_PER_S2 * dt;
                ball.resting = false;
            }
        }
        List<Contact> contacts = contacts();
        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            for (Contact contact : contacts) {
                contact.solve();
            }
        }
        for (Contact contact : contacts) {
            if (contact.pushed > 0) {
                contact.a.resting = true;
                if (contact.b != null) {
                    contact.b.resting = true;
                }
            }
        }
        for (Ball ball : balls) {
            if (!ball.awake) {
                continue;
            }
            if (ball.resting) {
                double rolling = 1 / (1 + dt / ROLL_SECONDS);
                for (int axis = 0; axis < 3; axis++) {
                    ball.velocity[axis] *= rolling;
                }
                ball.spin = scale(ball.spin, rolling);
            }
            for (int axis = 0; axis < 3; axis++) {
                ball.at[axis] += ball.velocity[axis] * dt;
            }
        }
        hives.advance(dt);
        pushApart();
        meetTheFloorAndTheWalls(landed);
        for (Ball ball : balls) {
            if (ball.awake) {
                double fastest = length(ball.velocity) + length(ball.spin) * ball.radius;
                ball.slowFor = fastest <= REST_SPEED_IN_PER_S ? ball.slowFor + dt : 0;
            }
        }
        for (Field.Hive hive : field.hives()) {
            if (!hives.tipping(hive) && fill(hive) >= SimHives.FULL) {
                hives.tip(hive);
            }
        }
    }

    private void wake() {
        Map<Field.Hive, Boolean> stirring = new LinkedHashMap<>();
        for (Field.Hive hive : field.hives()) {
            boolean moving = hives.tipping(hive);
            for (Ball ball : balls) {
                moving |= ball.slowFor < REST_SECONDS && hives.near(hive, ball.at, ball.radius + NEIGHBOURHOOD_IN);
            }
            stirring.put(hive, moving);
        }
        for (Ball ball : balls) {
            boolean nearAny = false, nearStirring = false;
            for (Field.Hive hive : field.hives()) {
                if (hives.near(hive, ball.at, ball.radius + NEIGHBOURHOOD_IN)) {
                    nearAny = true;
                    nearStirring |= stirring.get(hive);
                }
            }
            ball.awake = !nearAny || nearStirring;
            if (!ball.awake) {
                ball.velocity[0] = ball.velocity[1] = ball.velocity[2] = 0;
                ball.spin = new double[3];
            }
        }
    }

    private List<Contact> contacts() {
        List<Contact> contacts = new ArrayList<>();
        for (Ball ball : balls) {
            if (!ball.awake) {
                continue;
            }
            for (SimHives.Touch touch : hives.touching(ball.at, ball.radius)) {
                contacts.add(new Contact(ball, null, touch.normal, touch.velocity));
            }
        }
        for (int i = 0; i < balls.size(); i++) {
            for (int j = i + 1; j < balls.size(); j++) {
                Ball a = balls.get(i), b = balls.get(j);
                if (!a.awake && !b.awake) {
                    continue;
                }
                double[] between = sub(a.at, b.at);
                double apart = length(between);
                if (apart < a.radius + b.radius + CONTACT_TOLERANCE_IN && apart > 1e-9) {
                    contacts.add(new Contact(a, b, scale(between, 1 / apart), null));
                }
            }
        }
        return contacts;
    }

    private void pushApart() {
        for (int i = 0; i < PUSH_ITERATIONS; i++) {
            for (Ball ball : balls) {
                if (!ball.awake) {
                    continue;
                }
                SimHives.Touch deepest = null;
                for (SimHives.Touch touch : hives.touching(ball.at, ball.radius)) {
                    if (deepest == null || touch.depth > deepest.depth) {
                        deepest = touch;
                    }
                }
                if (deepest != null && deepest.depth > CONTACT_TOLERANCE_IN) {
                    move(ball, deepest.normal, deepest.depth - CONTACT_TOLERANCE_IN);
                }
            }
            for (int a = 0; a < balls.size(); a++) {
                for (int b = a + 1; b < balls.size(); b++) {
                    Ball one = balls.get(a), other = balls.get(b);
                    double ia = one.inverseMass(), ib = other.inverseMass();
                    if (ia + ib == 0) {
                        continue;
                    }
                    double[] between = sub(one.at, other.at);
                    double apart = length(between);
                    double over = one.radius + other.radius - apart;
                    if (over > CONTACT_TOLERANCE_IN && apart > 1e-9) {
                        double[] normal = scale(between, 1 / apart);
                        move(one, normal, (over - CONTACT_TOLERANCE_IN) * ia / (ia + ib));
                        move(other, normal, -(over - CONTACT_TOLERANCE_IN) * ib / (ia + ib));
                    }
                }
            }
        }
    }

    private static void move(Ball ball, double[] direction, double distance) {
        for (int axis = 0; axis < 3; axis++) {
            ball.at[axis] += direction[axis] * distance;
        }
    }

    private void meetTheFloorAndTheWalls(List<Landing> landed) {
        for (Ball ball : new ArrayList<>(balls)) {
            if (!ball.awake) {
                continue;
            }
            if (ball.at[2] < ball.radius) {
                ball.at[2] = ball.radius;
                if (-ball.velocity[2] < LANDING_SPEED_IN_PER_S) {
                    remove(ball.key);
                    landed.add(new Landing(
                            ball.key, false, ball.at.clone(), new double[] {ball.velocity[0], ball.velocity[1]}));
                    continue;
                }
                ball.velocity[2] = -ball.velocity[2] * BOUNCE;
            }
            double limit = field.size() / 2 - ball.radius;
            for (int axis = 0; axis < 2; axis++) {
                if (Math.abs(ball.at[axis]) <= limit) {
                    continue;
                }
                if (ball.at[2] - ball.radius > field.wallHeight()) {
                    remove(ball.key);
                    landed.add(new Landing(
                            ball.key, true, new double[] {ball.at[0], ball.at[1], ball.radius}, new double[2]));
                    break;
                }
                ball.at[axis] = Math.signum(ball.at[axis]) * limit;
                ball.velocity[axis] = -ball.velocity[axis] * BOUNCE;
            }
        }
    }
}
