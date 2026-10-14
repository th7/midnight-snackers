package org.firstinspires.ftc.teamcode.simcore;

final class GridTime {
    static final class PartOfAStep {
        private final double value;

        private PartOfAStep(double value) {
            this.value = value;
        }

        static PartOfAStep none() {
            return new PartOfAStep(0);
        }

        static Checked<PartOfAStep> of(double value) {
            if (!(value >= 0 && value < 1)) {
                return Checked.rejected("a part of a step is at least 0 and less than 1, not " + value);
            }
            return Checked.ok(new PartOfAStep(value));
        }

        double value() {
            return value;
        }
    }

    private final long steps;
    private final PartOfAStep part;
    private final Bodies.Step step;

    private GridTime(long steps, PartOfAStep part, Bodies.Step step) {
        this.steps = steps;
        this.part = part;
        this.step = step;
    }

    static GridTime at(long steps, Bodies.Step step) {
        return new GridTime(steps, PartOfAStep.none(), step);
    }

    static GridTime partWay(long steps, PartOfAStep part, Bodies.Step step) {
        return new GridTime(steps, part, step);
    }

    static Checked<GridTime> of(double seconds, Bodies.Step step) {
        double steps = seconds / step.seconds().value();
        double whole = StrictMath.floor(steps);
        if (!(whole >= Long.MIN_VALUE && whole < Long.MAX_VALUE)) {
            return Checked.rejected("a time on the grid of " + step.seconds() + " steps is a finite number of steps"
                    + " a long can count, not " + seconds + " s");
        }
        return PartOfAStep.of(steps - whole).map(part -> new GridTime((long) whole, part, step));
    }

    GridTime after(Seconds more) {
        double ahead = part.value() + more.value() / step.seconds().value();
        double whole = StrictMath.floor(ahead);
        if (!(whole < (double) Long.MAX_VALUE - steps)) {
            return at(Long.MAX_VALUE, step);
        }
        return PartOfAStep.of(ahead - whole)
                .map(rest -> new GridTime(steps + (long) whole, rest, step))
                .orElse(at(steps + (long) whole, step));
    }

    long steps() {
        return steps;
    }

    PartOfAStep part() {
        return part;
    }

    Bodies.Step step() {
        return step;
    }

    double seconds() {
        return (steps + part.value()) * step.seconds().value();
    }

    double secondsSince(long began) {
        return ((steps - began) + part.value()) * step.seconds().value();
    }

    @Override
    public String toString() {
        return "step " + steps + " and " + part.value() + " of a step";
    }
}
