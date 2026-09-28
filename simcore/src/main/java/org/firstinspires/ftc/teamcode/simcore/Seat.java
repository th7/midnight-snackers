package org.firstinspires.ftc.teamcode.simcore;

import java.util.Optional;

public sealed interface Seat<K> permits Seat.Seated, Seat.Unseated {
    static <K> Seat<K> seated() {
        return new Seated<>();
    }

    record Next<K>(Seat<K> seat, Optional<K> held) {}

    default Next<K> next(Optional<K> nested, boolean pushedByTheRobot, double speedMetresPerSecond) {
        return nested.map(ball -> {
                    Seat<K> seat = pushedByTheRobot ? new Unseated<>(ball) : this;
                    boolean offIt = seat.equals(new Unseated<>(ball));
                    boolean moving = speedMetresPerSecond > Flight.REST_SPEED_IN_PER_S * Length.METRES_PER_INCH;
                    return offIt && moving
                            ? new Next<>(seat, Optional.<K>empty())
                            : new Next<>(Seat.<K>seated(), Optional.of(ball));
                })
                .orElse(new Next<>(Seat.<K>seated(), Optional.empty()));
    }

    record Seated<K>() implements Seat<K> {}

    record Unseated<K>(K ball) implements Seat<K> {}
}
