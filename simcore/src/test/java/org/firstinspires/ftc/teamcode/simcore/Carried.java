package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

final class Carried implements Surroundings.Part {
    private final List<Collider> colliders;
    private final Vec3 pivot;
    private final Vec3 velocity;
    private final Vec3 spin;
    private final double starts;
    private final boolean inPlace;
    private final double seconds;

    private Carried(
            List<Collider> colliders,
            Vec3 pivot,
            Vec3 velocity,
            Vec3 spin,
            double starts,
            boolean inPlace,
            double seconds) {
        this.colliders = List.copyOf(colliders);
        this.pivot = pivot;
        this.velocity = velocity;
        this.spin = spin;
        this.starts = starts;
        this.inPlace = inPlace;
        this.seconds = seconds;
    }

    static Carried sliding(Collider collider, Vec3 velocity, double starts) {
        return new Carried(List.of(collider), Vec3.zero(), velocity, Vec3.zero(), starts, false, 0);
    }

    static Carried turning(Collider collider, Vec3 pivot, Vec3 spin, double starts) {
        return new Carried(List.of(collider), pivot, Vec3.zero(), spin, starts, false, 0);
    }

    static Carried spinningInPlace(Collider collider, Vec3 pivot, Vec3 spin, double starts) {
        return new Carried(List.of(collider), pivot, Vec3.zero(), spin, starts, true, 0);
    }

    static Collider box(String name, Vec3 least, Vec3 most) {
        return Valid.value(Collider.box(name, Surface.FRAME, least, most));
    }

    private double elapsed() {
        return inPlace ? 0 : StrictMath.max(0, seconds - starts);
    }

    private Vec3 offset() {
        return velocity.times(elapsed());
    }

    private double rate() {
        return StrictMath.sqrt(spin.dot(spin));
    }

    private Vec3 turned(Vec3 v, double angle) {
        double rate = rate();
        if (!(rate > 0)) {
            return v;
        }
        Vec3 axis = spin.times(1 / rate);
        double cos = StrictMath.cos(angle), sin = StrictMath.sin(angle);
        return v.times(cos).plus(axis.cross(v).times(sin)).plus(axis.times(axis.dot(v) * (1 - cos)));
    }

    private Vec3 local(Vec3 centre) {
        return pivot.plus(turned(centre.minus(pivot).minus(offset()), -rate() * elapsed()));
    }

    Vec3 placed(Vec3 local) {
        return pivot.plus(offset()).plus(turned(local.minus(pivot), rate() * elapsed()));
    }

    @Override
    public List<String> ids() {
        List<String> ids = new ArrayList<>();
        for (Collider collider : colliders) {
            ids.add(collider.name());
        }
        return ids;
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        Vec3 local = local(centre);
        for (Collider collider : colliders) {
            if (collider.reaches(local, by)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public double sweep() {
        if (inPlace) {
            return 0;
        }
        double reach = 0;
        for (Collider collider : colliders) {
            Vec3 least = collider.least(), most = collider.most();
            for (double x : new double[] {least.x(), most.x()}) {
                for (double y : new double[] {least.y(), most.y()}) {
                    for (double z : new double[] {least.z(), most.z()}) {
                        Vec3 arm = new Vec3(x, y, z).minus(pivot);
                        reach = StrictMath.max(reach, StrictMath.sqrt(arm.dot(arm)));
                    }
                }
            }
        }
        return StrictMath.sqrt(velocity.dot(velocity)) + rate() * reach;
    }

    @Override
    public boolean moving() {
        return seconds >= starts && (velocity.dot(velocity) > 0 || spin.dot(spin) > 0);
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        double r = radius.inches();
        Vec3 local = local(centre);
        List<Surroundings.Touch> touches = new ArrayList<>();
        for (Collider collider : colliders) {
            if (!collider.reaches(local, r + within)) {
                continue;
            }
            Colliders.Near near = collider.near(local, r);
            if (near.gap() > within) {
                continue;
            }
            Vec3 normal = turned(near.normal(), rate() * elapsed());
            Vec3 contact = centre.along(normal, -(r + near.gap()));
            boolean moving = moving();
            touches.add(Valid.value(Surroundings.Touch.of(
                    collider.name(),
                    collider.surface(),
                    normal,
                    near.gap(),
                    moving ? velocity.plus(spin.cross(contact.minus(pivot.plus(offset())))) : Vec3.zero(),
                    moving ? spin : Vec3.zero())));
        }
        return touches;
    }

    @Override
    public Surroundings.Part at(double seconds) {
        return new Carried(colliders, pivot, velocity, spin, starts, inPlace, seconds);
    }
}
