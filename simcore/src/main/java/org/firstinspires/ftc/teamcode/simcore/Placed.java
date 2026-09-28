package org.firstinspires.ftc.teamcode.simcore;

public sealed interface Placed permits Placed.AsGiven, Placed.Moved {
    Vec2 from(Vec2 given);

    Placed after(Placed earlier);

    record AsGiven() implements Placed {
        @Override
        public Vec2 from(Vec2 given) {
            return given;
        }

        @Override
        public Placed after(Placed earlier) {
            return earlier;
        }
    }

    record Moved(Vec2 to) implements Placed {
        @Override
        public Vec2 from(Vec2 given) {
            return to;
        }

        @Override
        public Placed after(Placed earlier) {
            return this;
        }
    }
}
