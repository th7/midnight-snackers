package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class Contacts {
    public static final int ITERATIONS = 20;

    private Contacts() {}

    public record Body(String name, Pollen shape, Vec3 velocity, Vec3 spin) {}

    public sealed interface Against {
        record Fixed(String name, Surface surface) implements Against {}

        record Ball(Body body) implements Against {}
    }

    public record Pressing(Body ball, Against against, Vec3 normal, boolean fresh) {}

    public record Resolved(List<Body> bodies, List<Hit> hits) {}

    public record Hit(
            String ball,
            String against,
            Surface surface,
            Vec3 normal,
            double approach,
            double slip,
            double restitution,
            boolean gripped,
            double normalImpulse) {}

    public static Resolved resolve(Surfaces surfaces, List<Pressing> pressing) {
        List<Taking> taking = new ArrayList<>();
        List<Contact> contacts = new ArrayList<>();
        for (Pressing each : pressing) {
            Moving a = joined(taking, each.ball());
            Vec3 normal = each.normal();
            double length = magnitude(normal);
            boolean directed = length > 0 && Double.isFinite(length);
            Vec3 unit = normal.times(1 / length);
            Against against = each.against();
            if (against instanceof Against.Fixed fixed && directed) {
                contacts.add(new Contact(
                        surfaces,
                        each,
                        a,
                        Moving.immovable(),
                        unit,
                        fixed.name(),
                        fixed.surface(),
                        surfaces.rollingResistance()));
            }
            if (against instanceof Against.Ball ball) {
                Moving b = joined(taking, ball.body());
                if (directed && a != b) {
                    contacts.add(
                            new Contact(surfaces, each, a, b, unit, ball.body().name(), Surface.PIECE, 0));
                }
            }
        }
        for (int i = 0; i < ITERATIONS; i++) {
            for (Contact contact : contacts) {
                contact.solve();
            }
        }
        for (Contact contact : contacts) {
            contact.twist();
            contact.resistRolling();
        }
        List<Body> bodies = new ArrayList<>();
        for (Taking each : taking) {
            Body body = each.body();
            bodies.add(new Body(body.name(), body.shape(), each.moving().velocity, each.moving().spin));
        }
        List<Hit> hits = new ArrayList<>();
        for (Contact contact : contacts) {
            if (contact.bounced) {
                hits.add(contact.hit());
            }
        }
        return new Resolved(Collections.unmodifiableList(bodies), Collections.unmodifiableList(hits));
    }

    private static double magnitude(Vec3 vector) {
        return StrictMath.sqrt(vector.dot(vector));
    }

    private static Vec3 across(Vec3 vector, Vec3 normal) {
        return vector.minus(normal.times(vector.dot(normal)));
    }

    private static Moving joined(List<Taking> taking, Body body) {
        for (Taking each : taking) {
            if (each.body().name().equals(body.name())) {
                return each.moving();
            }
        }
        Moving moving = Moving.of(body);
        taking.add(new Taking(body, moving));
        return moving;
    }

    private record Taking(Body body, Moving moving) {}

    private static final class Moving {
        private final double inverseMass;
        private final double inverseInertia;
        private final double radius;
        private Vec3 velocity;
        private Vec3 spin;

        private Moving(double inverseMass, double inverseInertia, double radius, Vec3 velocity, Vec3 spin) {
            this.inverseMass = inverseMass;
            this.inverseInertia = inverseInertia;
            this.radius = radius;
            this.velocity = velocity;
            this.spin = spin;
        }

        static Moving of(Body body) {
            Pollen shape = body.shape();
            double kilograms = shape.kilograms(), radius = shape.radius();
            double inertia = shape.inertiaFactor() * kilograms * radius * radius;
            return new Moving(1 / kilograms, 1 / inertia, radius, body.velocity(), body.spin());
        }

        static Moving immovable() {
            return new Moving(0, 0, 0, Vec3.zero(), Vec3.zero());
        }

        double inverseAcross() {
            return inverseMass + radius * radius * inverseInertia;
        }

        Vec3 at(Vec3 arm) {
            return velocity.plus(spin.cross(arm));
        }
    }

    private static final class Contact {
        private final Moving a;
        private final Moving b;
        private final Vec3 normal;
        private final Vec3 armA;
        private final Vec3 armB;
        private final String ball;
        private final String against;
        private final Surface surface;
        private final double approach;
        private final double slip;
        private final boolean bounced;
        private final double restitution;
        private final double aimNormal;
        private final Vec3 aimAcross;
        private final double friction;
        private final double lever;
        private final double rollingPerPush;
        private final double normalMass;
        private final double acrossMass;
        private double pushed;
        private Vec3 rubbed = Vec3.zero();
        private boolean fitted = true;

        Contact(
                Surfaces surfaces,
                Pressing pressing,
                Moving a,
                Moving b,
                Vec3 normal,
                String against,
                Surface surface,
                double rollingResistance) {
            this.a = a;
            this.b = b;
            this.normal = normal;
            this.armA = normal.times(-a.radius);
            this.armB = normal.times(b.radius);
            this.ball = pressing.ball().name();
            this.against = against;
            this.surface = surface;
            Vec3 meeting = meeting();
            double closing = meeting.dot(normal);
            Vec3 slipping = across(meeting, normal);
            this.approach = -closing;
            this.slip = magnitude(slipping);
            this.bounced = pressing.fresh() && closing < -surfaces.threshold();
            Bounce bounce = surfaces.of(surface);
            this.restitution = bounced ? surfaces.restitution(surface, approach) : 0;
            this.aimNormal = bounced ? restitution * approach : 0;
            this.aimAcross = bounced ? slipping.times(-bounce.grip()) : Vec3.zero();
            this.friction = bounce.friction();
            this.lever = bounced ? surfaces.impactTwist() : surfaces.spinTwist();
            this.rollingPerPush = bounced ? 0 : rollingResistance * a.inverseAcross() / a.radius;
            this.normalMass = 1 / (a.inverseMass + b.inverseMass);
            this.acrossMass = 1 / (a.inverseAcross() + b.inverseAcross());
        }

        Vec3 meeting() {
            return a.at(armA).minus(b.at(armB));
        }

        void solve() {
            double closing = meeting().dot(normal);
            double total = StrictMath.max(0, pushed + normalMass * (aimNormal - closing));
            push(normal.times(total - pushed));
            pushed = total;

            Vec3 slipping = across(meeting(), normal);
            Vec3 wanted = rubbed.plus(aimAcross.minus(slipping).times(acrossMass));
            double most = friction * pushed;
            double size = magnitude(wanted);
            fitted = !(size > most);
            Vec3 allowed = fitted ? wanted : wanted.times(most / size);
            push(allowed.minus(rubbed));
            rubbed = allowed;
        }

        void push(Vec3 impulse) {
            a.velocity = a.velocity.along(impulse, a.inverseMass);
            a.spin = a.spin.along(armA.cross(impulse), a.inverseInertia);
            b.velocity = b.velocity.along(impulse, -b.inverseMass);
            b.spin = b.spin.along(armB.cross(impulse), -b.inverseInertia);
        }

        void twist() {
            double most = lever * pushed;
            if (!(most > 0)) {
                return;
            }
            double about = a.spin.minus(b.spin).dot(normal);
            double stopping = StrictMath.abs(about) / (a.inverseInertia + b.inverseInertia);
            double turned = StrictMath.copySign(StrictMath.min(most, stopping), about);
            a.spin = a.spin.along(normal, -turned * a.inverseInertia);
            b.spin = b.spin.along(normal, turned * b.inverseInertia);
        }

        void resistRolling() {
            double most = rollingPerPush * pushed;
            if (!(most > 0)) {
                return;
            }
            Vec3 rolling = across(a.spin, normal);
            a.spin = a.spin.minus(rolling.times(StrictMath.min(1, most / magnitude(rolling))));
        }

        Hit hit() {
            return new Hit(ball, against, surface, normal, approach, slip, restitution, fitted, pushed);
        }
    }
}
