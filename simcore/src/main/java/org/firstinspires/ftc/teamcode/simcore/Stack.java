package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public final class Stack<K> {
    private final Field.Flower flower;
    private final List<Standing<K>> fromTheBottom;

    private record Standing<K>(K key, Length radius, double z, double vz) {
        double top() {
            return z + radius.inches();
        }
    }

    public record Settled<K>(Stack<K> stack, List<K> leaving) {}

    private Stack(Field.Flower flower, List<Standing<K>> fromTheBottom) {
        this.flower = flower;
        this.fromTheBottom = Collections.unmodifiableList(new ArrayList<>(fromTheBottom));
    }

    public static <K> Stack<K> in(Field.Flower flower) {
        return new Stack<>(flower, List.of());
    }

    public Field.Flower flower() {
        return flower;
    }

    public int size() {
        return fromTheBottom.size();
    }

    public List<K> fromTheBottom() {
        List<K> keys = new ArrayList<>();
        for (Standing<K> standing : fromTheBottom) {
            keys.add(standing.key());
        }
        return keys;
    }

    public Optional<Vec3> at(K key) {
        Optional<Vec3> at = Optional.empty();
        for (Standing<K> standing : fromTheBottom) {
            if (standing.key().equals(key)) {
                at = Optional.of(new Vec3(flower.axis().x(), flower.axis().y(), standing.z()));
            }
        }
        return at;
    }

    public Stack<K> with(K key, Length radius, double z) {
        Standing<K> added = new Standing<>(key, radius, z, 0);
        List<Standing<K>> balls = new ArrayList<>();
        boolean placed = false;
        for (Standing<K> standing : fromTheBottom) {
            if (!placed && !(standing.z() < z)) {
                balls.add(added);
                placed = true;
            }
            balls.add(standing);
        }
        if (!placed) {
            balls.add(added);
        }
        return new Stack<>(flower, balls);
    }

    public Stack<K> without(K key) {
        List<Standing<K>> balls = new ArrayList<>();
        for (Standing<K> standing : fromTheBottom) {
            if (!standing.key().equals(key)) {
                balls.add(standing);
            }
        }
        return new Stack<>(flower, balls);
    }

    public Settled<K> rested(List<Rolling<K>> rolling) {
        double under = floor(rolling);
        List<Standing<K>> standing = new ArrayList<>();
        List<K> leaving = new ArrayList<>();
        for (Standing<K> ball : fromTheBottom) {
            Standing<K> rests = new Standing<>(
                    ball.key(), ball.radius(), under + ball.radius().inches(), 0);
            under = settle(rests, standing, leaving);
        }
        return new Settled<>(new Stack<>(flower, standing), leaving);
    }

    public Settled<K> after(Seconds dt, List<Rolling<K>> rolling) {
        double under = floor(rolling);
        List<Standing<K>> standing = new ArrayList<>();
        List<K> leaving = new ArrayList<>();
        for (Standing<K> ball : fromTheBottom) {
            double resting = under + ball.radius().inches();
            double z = ball.z();
            double vz = ball.vz();
            if (z > resting) {
                vz -= Flight.GRAVITY_IN_PER_S2 * dt.value();
                z += vz * dt.value();
            }
            if (z <= resting) {
                z = resting;
                vz = -vz < Flight.LANDING_SPEED_IN_PER_S ? 0 : -vz * Flight.BOUNCE;
            }
            under = settle(new Standing<>(ball.key(), ball.radius(), z, vz), standing, leaving);
        }
        return new Settled<>(new Stack<>(flower, standing), leaving);
    }

    private double settle(Standing<K> ball, List<Standing<K>> standing, List<K> leaving) {
        if (ball.vz() == 0 && ball.top() <= flower.lip()) {
            leaving.add(ball.key());
        } else {
            standing.add(ball);
        }
        return ball.top();
    }

    private double floor(List<Rolling<K>> rolling) {
        double top = 0;
        for (Rolling<K> ball : rolling) {
            if (flower.standsIn(ball.at().x(), ball.at().y())) {
                top = Math.max(top, 2 * ball.radius().inches());
            }
        }
        return top;
    }
}
