package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;
import java.util.function.Function;

public record PerWheel<T>(T leftFront, T rightFront, T leftBack, T rightBack) {
    public static <T> PerWheel<T> all(T each) {
        return new PerWheel<>(each, each, each, each);
    }

    public static <T> PerWheel<T> each(Function<Wheel, ? extends T> of) {
        return new PerWheel<>(
                of.apply(Wheel.LEFT_FRONT),
                of.apply(Wheel.RIGHT_FRONT),
                of.apply(Wheel.LEFT_BACK),
                of.apply(Wheel.RIGHT_BACK));
    }

    public static <T> Checked<PerWheel<T>> allOf(PerWheel<Checked<T>> checked) {
        return checked.leftFront.then(leftFront ->
                checked.rightFront.then(rightFront -> checked.leftBack.then(leftBack -> checked.rightBack.map(
                        rightBack -> new PerWheel<>(leftFront, rightFront, leftBack, rightBack)))));
    }

    public T of(Wheel wheel) {
        return switch (wheel) {
            case LEFT_FRONT -> leftFront;
            case RIGHT_FRONT -> rightFront;
            case LEFT_BACK -> leftBack;
            case RIGHT_BACK -> rightBack;
        };
    }

    public PerWheel<T> with(Wheel wheel, T value) {
        return new PerWheel<>(
                wheel == Wheel.LEFT_FRONT ? value : leftFront,
                wheel == Wheel.RIGHT_FRONT ? value : rightFront,
                wheel == Wheel.LEFT_BACK ? value : leftBack,
                wheel == Wheel.RIGHT_BACK ? value : rightBack);
    }

    public <R> PerWheel<R> map(Function<? super T, ? extends R> f) {
        return new PerWheel<>(f.apply(leftFront), f.apply(rightFront), f.apply(leftBack), f.apply(rightBack));
    }

    public List<T> inOrder() {
        return List.of(leftFront, rightFront, leftBack, rightBack);
    }
}
