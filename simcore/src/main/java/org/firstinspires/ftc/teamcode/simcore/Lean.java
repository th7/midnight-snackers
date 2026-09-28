package org.firstinspires.ftc.teamcode.simcore;

import java.util.Optional;

public sealed interface Lean permits Lean.Resting, Lean.Tipping {
    double TIP_SECONDS = 1;

    double degrees();

    double degreesPerSecond();

    Lean after(Seconds elapsed);

    Lean tipped();

    Optional<Tilt> tippingTo();

    record Resting(Tilt tilt) implements Lean {
        @Override
        public double degrees() {
            return tilt.degrees();
        }

        @Override
        public double degreesPerSecond() {
            return 0;
        }

        @Override
        public Lean after(Seconds elapsed) {
            return this;
        }

        @Override
        public Lean tipped() {
            return new Tipping(tilt, tilt.opposite(), Seconds.zero());
        }

        @Override
        public Optional<Tilt> tippingTo() {
            return Optional.empty();
        }
    }

    record Tipping(Tilt from, Tilt to, Seconds elapsed) implements Lean {
        @Override
        public double degrees() {
            double a = from.degrees(), b = to.degrees();
            return a + (b - a) * (1 - Math.cos(Math.PI * elapsed.value() / TIP_SECONDS)) / 2;
        }

        @Override
        public double degreesPerSecond() {
            double a = from.degrees(), b = to.degrees();
            return (b - a) * Math.PI / (2 * TIP_SECONDS) * Math.sin(Math.PI * elapsed.value() / TIP_SECONDS);
        }

        @Override
        public Lean after(Seconds more) {
            Seconds now = elapsed.plus(more);
            if (now.value() >= TIP_SECONDS) {
                return new Resting(to);
            }
            return new Tipping(from, to, now);
        }

        @Override
        public Lean tipped() {
            return this;
        }

        @Override
        public Optional<Tilt> tippingTo() {
            return Optional.of(to);
        }
    }
}
