package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public final class World {
    public static final double LONGEST_STEP_SECONDS = 0.01;

    public record Piece(
            String name, Field.Kind kind, Optional<String> alliance, Pollen shape, Vec3 at, boolean fixed) {}

    private final Colliders colliders;
    private final Air air;
    private final Surfaces surfaces;
    private final Seconds step;
    private final Seconds longest;
    private final List<Piece> pieces;

    private World(Colliders colliders, Air air, Surfaces surfaces, Seconds step, Seconds longest, List<Piece> pieces) {
        this.colliders = colliders;
        this.air = air;
        this.surfaces = surfaces;
        this.step = step;
        this.longest = longest;
        this.pieces = Collections.unmodifiableList(new ArrayList<>(pieces));
    }

    public static Checked<World> of(
            Colliders colliders, Air air, Surfaces surfaces, Seconds step, Seconds longest, List<Piece> pieces) {
        if (!(step.value() > 0 && step.value() <= LONGEST_STEP_SECONDS)) {
            return Checked.rejected(
                    "a shot's step is more than 0 and at most " + LONGEST_STEP_SECONDS + " s, not " + step.value());
        }
        if (!(longest.value() > 0)) {
            return Checked.rejected("a shot flies for some time, not " + longest.value() + " s");
        }
        Set<String> named = new HashSet<>();
        for (Piece piece : pieces) {
            if (piece.name().isBlank() || !named.add(piece.name())) {
                return Checked.rejected("every piece has a name of its own, and '" + piece.name() + "' is not one");
            }
            if (!piece.at().isFinite()) {
                return Checked.rejected(piece.name() + " is somewhere on the field, not at " + piece.at());
            }
        }
        return Checked.ok(new World(colliders, air, surfaces, step, longest, pieces));
    }

    public Colliders colliders() {
        return colliders;
    }

    public Air air() {
        return air;
    }

    public Surfaces surfaces() {
        return surfaces;
    }

    public Seconds step() {
        return step;
    }

    public Seconds longest() {
        return longest;
    }

    public List<Piece> pieces() {
        return pieces;
    }
}
