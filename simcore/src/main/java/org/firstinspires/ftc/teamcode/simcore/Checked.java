package org.firstinspires.ftc.teamcode.simcore;

import java.util.function.Function;

public sealed interface Checked<T> permits Checked.Ok, Checked.Rejected {
    static <T> Checked<T> ok(T value) {
        return new Ok<>(value);
    }

    static <T> Checked<T> rejected(String rule) {
        return new Rejected<>(rule);
    }

    <R> R fold(Function<? super T, ? extends R> ok, Function<String, ? extends R> rejected);

    default <R> Checked<R> map(Function<? super T, ? extends R> f) {
        return fold(value -> ok(f.apply(value)), Checked::rejected);
    }

    default <R> Checked<R> then(Function<? super T, Checked<R>> f) {
        return fold(f, Checked::rejected);
    }

    default T orElse(T otherwise) {
        return fold(value -> value, rule -> otherwise);
    }

    record Ok<T>(T value) implements Checked<T> {
        @Override
        public <R> R fold(Function<? super T, ? extends R> ok, Function<String, ? extends R> rejected) {
            return ok.apply(value);
        }
    }

    record Rejected<T>(String rule) implements Checked<T> {
        @Override
        public <R> R fold(Function<? super T, ? extends R> ok, Function<String, ? extends R> rejected) {
            return rejected.apply(rule);
        }
    }
}
