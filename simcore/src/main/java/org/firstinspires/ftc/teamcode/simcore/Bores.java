package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

final class Bores implements Surroundings.Part {
    static final double STILL_WITHIN_IN = 0.02;

    private record Standing(String id, Vec3 centre, Length radius) {
        double reach(Vec3 from) {
            Vec3 apart = from.minus(centre);
            return StrictMath.sqrt(apart.dot(apart));
        }
    }

    private final List<Field.Flower> flowers;
    private final Map<String, Colliders.BasePlate> plates;
    private final List<Standing> standing;
    private final List<String> ids;

    private Bores(List<Field.Flower> flowers, Map<String, Colliders.BasePlate> plates, List<Standing> standing) {
        this.flowers = flowers;
        this.plates = plates;
        this.standing = Collections.unmodifiableList(standing);
        List<String> ids = new ArrayList<>();
        for (Standing one : standing) {
            ids.add(one.id());
        }
        this.ids = Collections.unmodifiableList(ids);
    }

    static Bores of(Field field) {
        return new Bores(field.flowers(), Colliders.basePlatesOf(field), List.of());
    }

    <K> Bores holding(Collection<Stack<K>> stacks, List<Rolling<K>> rolling) {
        List<Standing> now = new ArrayList<>();
        for (Field.Flower flower : flowers) {
            Colliders.BasePlate plate = plates.get(flower.name());
            double floor = plate == null ? 0 : plate.top();
            List<Standing> inThisBore = new ArrayList<>();
            for (Rolling<K> ball : rolling) {
                if (flower.standsIn(ball.at().x(), ball.at().y())) {
                    inThisBore.add(standing(
                            flower,
                            inThisBore,
                            new Vec3(
                                    ball.at().x(),
                                    ball.at().y(),
                                    floor + ball.radius().inches()),
                            ball.radius()));
                }
            }
            for (Stack<K> stack : stacks) {
                if (stack.flower().name().equals(flower.name())) {
                    for (Stack.Standing<K> ball : stack.standing()) {
                        inThisBore.add(standing(
                                flower,
                                inThisBore,
                                new Vec3(flower.axis().x(), flower.axis().y(), floor + ball.z()),
                                ball.radius()));
                    }
                }
            }
            now.addAll(inThisBore);
        }
        return new Bores(flowers, plates, now);
    }

    private static Standing standing(Field.Flower flower, List<Standing> before, Vec3 centre, Length radius) {
        return new Standing(flower.name() + " / standing <" + (before.size() + 1) + ">", centre, radius);
    }

    boolean empty() {
        return standing.isEmpty();
    }

    boolean standsAsIn(Bores before) {
        if (standing.size() != before.standing.size()) {
            return false;
        }
        Map<String, Standing> was = new HashMap<>();
        for (Standing then : before.standing) {
            was.put(then.id(), then);
        }
        for (Standing now : standing) {
            Standing then = was.get(now.id());
            if (then == null || !now.radius().equals(then.radius()) || !(now.reach(then.centre()) <= STILL_WITHIN_IN)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public List<String> ids() {
        return ids;
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        for (Standing one : standing) {
            if (one.reach(centre) <= by + one.radius().inches()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public double sweep() {
        return 0;
    }

    @Override
    public boolean moving() {
        return false;
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        List<Surroundings.Touch> touches = new ArrayList<>();
        for (Standing one : standing) {
            double apart = one.reach(centre);
            double gap = apart - radius.inches() - one.radius().inches();
            if (gap <= within) {
                Vec3 away = centre.minus(one.centre()).times(1 / apart);
                Surroundings.Touch.of(
                                one.id(),
                                Surface.PIECE,
                                StrictMath.abs(away.dot(away) - 1) <= Surroundings.Touch.UNIT_WITHIN
                                        ? away
                                        : new Vec3(0, 0, 1),
                                gap,
                                Vec3.zero(),
                                Vec3.zero())
                        .fold(touch -> touches.add(touch), rule -> false);
            }
        }
        return touches;
    }

    @Override
    public boolean squeezes() {
        return false;
    }

    @Override
    public List<Surroundings.Overlap> squeezing(Vec3 centre, Length radius) {
        return List.of();
    }

    @Override
    public Surroundings.Part at(GridTime time) {
        return this;
    }
}
