package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

final class Joined implements Surroundings.Part {
    private final List<Surroundings.Part> parts;

    private Joined(List<Surroundings.Part> parts) {
        this.parts = List.copyOf(parts);
    }

    static Joined of(Surroundings.Part... parts) {
        return new Joined(List.of(parts));
    }

    @Override
    public List<String> ids() {
        List<String> ids = new ArrayList<>();
        for (Surroundings.Part part : parts) {
            ids.addAll(part.ids());
        }
        return ids;
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        for (Surroundings.Part part : parts) {
            if (part.reaches(centre, by)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public double sweep() {
        double sweep = 0;
        for (Surroundings.Part part : parts) {
            if (part.moving()) {
                sweep = StrictMath.max(sweep, part.sweep());
            }
        }
        return sweep;
    }

    @Override
    public boolean moving() {
        for (Surroundings.Part part : parts) {
            if (part.moving()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        List<Surroundings.Touch> touches = new ArrayList<>();
        for (Surroundings.Part part : parts) {
            if (part.reaches(centre, radius.inches() + within)) {
                touches.addAll(part.near(centre, radius, within));
            }
        }
        return touches;
    }

    @Override
    public Surroundings.Part at(double seconds) {
        List<Surroundings.Part> then = new ArrayList<>();
        for (Surroundings.Part part : parts) {
            then.add(part.at(seconds));
        }
        return new Joined(then);
    }
}
