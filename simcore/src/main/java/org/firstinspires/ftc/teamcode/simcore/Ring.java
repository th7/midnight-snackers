package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public final class Ring<T> {
    public record Edge<T>(T from, T to) {}

    private final T first;
    private final List<T> rest;

    private Ring(T first, List<T> rest) {
        this.first = first;
        this.rest = Collections.unmodifiableList(new ArrayList<>(rest));
    }

    public static <T> Ring<T> of(T first, List<T> rest) {
        return new Ring<>(first, rest);
    }

    public static <T> Checked<Ring<T>> of(List<T> all) {
        Optional<T> first = Optional.empty();
        List<T> rest = new ArrayList<>();
        for (T element : all) {
            if (first.isPresent()) {
                rest.add(element);
            } else {
                first = Optional.of(element);
            }
        }
        return first.map(head -> Checked.ok(new Ring<>(head, rest)))
                .orElse(Checked.rejected("a ring has at least one element"));
    }

    public <R> Ring<R> map(Function<? super T, ? extends R> f) {
        List<R> mapped = new ArrayList<>();
        for (T element : rest) {
            mapped.add(f.apply(element));
        }
        return new Ring<>(f.apply(first), mapped);
    }

    public T first() {
        return first;
    }

    public List<T> all() {
        List<T> all = new ArrayList<>();
        all.add(first);
        all.addAll(rest);
        return Collections.unmodifiableList(all);
    }

    public int size() {
        return 1 + rest.size();
    }

    public List<Edge<T>> edges() {
        List<Edge<T>> edges = new ArrayList<>();
        T from = first;
        for (T to : rest) {
            edges.add(new Edge<>(from, to));
            from = to;
        }
        edges.add(new Edge<>(from, first));
        return Collections.unmodifiableList(edges);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Ring<?> that && all().equals(that.all());
    }

    @Override
    public int hashCode() {
        return all().hashCode();
    }

    @Override
    public String toString() {
        return all().toString();
    }
}
