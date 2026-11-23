package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public final class Directions {
    private final EnumSet<Mechanism> reversed;

    private Directions(EnumSet<Mechanism> reversed) {
        this.reversed = reversed;
    }

    public static Directions reginalds() {
        return new Directions(EnumSet.noneOf(Mechanism.class));
    }

    public Directions with(Mechanism mechanism, boolean reversing) {
        EnumSet<Mechanism> now = EnumSet.copyOf(reversed);
        if (reversing) {
            now.add(mechanism);
        } else {
            now.remove(mechanism);
        }
        return new Directions(now);
    }

    public boolean reversed(Mechanism mechanism) {
        return reversed.contains(mechanism);
    }

    public boolean asBuilt() {
        return reversed.isEmpty();
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Directions that && reversed.equals(that.reversed);
    }

    @Override
    public int hashCode() {
        return reversed.hashCode();
    }

    @Override
    public String toString() {
        List<String> said = new ArrayList<>();
        for (Mechanism mechanism : Mechanism.values()) {
            said.add(mechanism.said(reversed(mechanism)));
        }
        return String.join(", ", said);
    }
}
