package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public final class World {
    public static final double LONGEST_STEP_SECONDS = Bodies.Step.LONGEST_SECONDS;

    public record Piece(String name, Field.Kind kind, Optional<String> alliance, Ball shape, Vec3 at, boolean fixed) {}

    private final Colliders colliders;
    private final Air air;
    private final Surfaces surfaces;
    private final Bodies.Step step;
    private final Seconds longest;
    private final List<Piece> pieces;
    private final Bodies.Pieces bodies;

    private World(
            Colliders colliders,
            Air air,
            Surfaces surfaces,
            Bodies.Step step,
            Seconds longest,
            List<Piece> pieces,
            Bodies.Pieces bodies) {
        this.colliders = colliders;
        this.air = air;
        this.surfaces = surfaces;
        this.step = step;
        this.longest = longest;
        this.pieces = Collections.unmodifiableList(new ArrayList<>(pieces));
        this.bodies = bodies;
    }

    public static Checked<World> of(
            Colliders colliders, Air air, Surfaces surfaces, Seconds step, Seconds longest, List<Piece> pieces) {
        return Bodies.Step.of(step).then(stepping -> of(colliders, air, surfaces, stepping, longest, pieces));
    }

    private static Checked<World> of(
            Colliders colliders, Air air, Surfaces surfaces, Bodies.Step step, Seconds longest, List<Piece> pieces) {
        if (!(longest.value() > 0)) {
            return Checked.rejected("a shot flies for some time, not " + longest.value() + " s");
        }
        Checked<Bodies.Pieces> laid = Checked.ok(Bodies.Pieces.none());
        for (Piece piece : pieces) {
            laid = laid.then(so ->
                    entryOf(piece).then(entry -> so.with(entry).fold(Checked::ok, rule -> notItsOwn(piece))));
        }
        return laid.map(bodies -> new World(colliders, air, surfaces, step, longest, pieces, bodies));
    }

    private static Checked<Bodies.Entry> entryOf(Piece piece) {
        Bodies.Start start = piece.fixed() ? new Bodies.Start.Fixed() : new Bodies.Start.Asleep();
        return Bodies.Name.of(piece.name())
                .fold(Checked::ok, rule -> World.<Bodies.Name>notItsOwn(piece))
                .then(name -> Bodies.Motion.of(piece.at(), Vec3.zero(), Vec3.zero())
                        .fold(
                                motion -> Checked.ok(new Bodies.Entry(name, piece.shape(), motion, start)),
                                rule -> Checked.rejected(
                                        piece.name() + " is somewhere on the field, not at " + piece.at())));
    }

    private static <T> Checked<T> notItsOwn(Piece piece) {
        return Checked.rejected("every piece has a name of its own, and '" + piece.name() + "' is not one");
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
        return step.seconds();
    }

    Bodies.Step stepping() {
        return step;
    }

    Bodies.Pieces bodies() {
        return bodies;
    }

    public Seconds longest() {
        return longest;
    }

    public List<Piece> pieces() {
        return pieces;
    }
}
