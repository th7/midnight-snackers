package org.firstinspires.ftc.teamcode.simcore;

public final class Surfaces {
    public static final double REFERENCE_APPROACH_IN_PER_S = 244;
    public static final double LOSS_SPAN_IN_PER_S = 100;
    public static final double LEAST_RESTITUTION = 0.1;
    public static final double MOST_RESTITUTION = 0.9;

    private final Bounce floor;
    private final Bounce wall;
    private final Bounce frame;
    private final Bounce flower;
    private final Bounce cell;
    private final Bounce piece;
    private final double loss;
    private final double threshold;
    private final double impactTwist;
    private final double spinTwist;
    private final double rollingResistance;

    private Surfaces(
            Bounce floor,
            Bounce wall,
            Bounce frame,
            Bounce flower,
            Bounce cell,
            Bounce piece,
            double loss,
            double threshold,
            double impactTwist,
            double spinTwist,
            double rollingResistance) {
        this.floor = floor;
        this.wall = wall;
        this.frame = frame;
        this.flower = flower;
        this.cell = cell;
        this.piece = piece;
        this.loss = loss;
        this.threshold = threshold;
        this.impactTwist = impactTwist;
        this.spinTwist = spinTwist;
        this.rollingResistance = rollingResistance;
    }

    public static Checked<Surfaces> of(
            Bounce floor,
            Bounce wall,
            Bounce frame,
            Bounce flower,
            Bounce cell,
            Bounce piece,
            double loss,
            double threshold,
            double impactTwist,
            double spinTwist,
            double rollingResistance) {
        if (!(loss >= 0 && Double.isFinite(loss))) {
            return Checked.rejected("the bounce loss is a finite number not below 0 per 100 in/s, not " + loss);
        }
        if (!(threshold > 0 && Double.isFinite(threshold))) {
            return Checked.rejected("the bounce threshold is a positive, finite speed in in/s, not " + threshold);
        }
        if (!(impactTwist >= 0 && Double.isFinite(impactTwist))) {
            return Checked.rejected("the impact twist is a finite lever not below 0 inches, not " + impactTwist);
        }
        if (!(spinTwist >= 0 && Double.isFinite(spinTwist))) {
            return Checked.rejected("the spin twist is a finite lever not below 0 inches, not " + spinTwist);
        }
        if (!(rollingResistance >= 0 && Double.isFinite(rollingResistance))) {
            return Checked.rejected("the rolling resistance is a finite number not below 0, not " + rollingResistance);
        }
        return Checked.ok(new Surfaces(
                floor, wall, frame, flower, cell, piece, loss, threshold, impactTwist, spinTwist, rollingResistance));
    }

    public Bounce of(Surface surface) {
        return switch (surface) {
            case FLOOR -> floor;
            case WALL -> wall;
            case FRAME -> frame;
            case FLOWER -> flower;
            case CELL -> cell;
            case PIECE -> piece;
        };
    }

    public double restitution(Surface surface, double approach) {
        double reference = of(surface).restitution();
        double slowed =
                reference - loss * (StrictMath.abs(approach) - REFERENCE_APPROACH_IN_PER_S) / LOSS_SPAN_IN_PER_S;
        double least = StrictMath.min(reference, LEAST_RESTITUTION);
        double most = StrictMath.max(reference, MOST_RESTITUTION);
        return StrictMath.min(most, StrictMath.max(least, slowed));
    }

    public double loss() {
        return loss;
    }

    public double threshold() {
        return threshold;
    }

    public double impactTwist() {
        return impactTwist;
    }

    public double spinTwist() {
        return spinTwist;
    }

    public double rollingResistance() {
        return rollingResistance;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Surfaces that
                && floor.equals(that.floor)
                && wall.equals(that.wall)
                && frame.equals(that.frame)
                && flower.equals(that.flower)
                && cell.equals(that.cell)
                && piece.equals(that.piece)
                && Double.compare(loss, that.loss) == 0
                && Double.compare(threshold, that.threshold) == 0
                && Double.compare(impactTwist, that.impactTwist) == 0
                && Double.compare(spinTwist, that.spinTwist) == 0
                && Double.compare(rollingResistance, that.rollingResistance) == 0;
    }

    @Override
    public int hashCode() {
        int hash = floor.hashCode();
        hash = hash * 31 + wall.hashCode();
        hash = hash * 31 + frame.hashCode();
        hash = hash * 31 + flower.hashCode();
        hash = hash * 31 + cell.hashCode();
        hash = hash * 31 + piece.hashCode();
        hash = hash * 31 + Double.hashCode(loss);
        hash = hash * 31 + Double.hashCode(threshold);
        hash = hash * 31 + Double.hashCode(impactTwist);
        hash = hash * 31 + Double.hashCode(spinTwist);
        return hash * 31 + Double.hashCode(rollingResistance);
    }

    @Override
    public String toString() {
        return "the floor " + floor + "; the walls " + wall + "; the frame " + frame + "; the flowers " + flower
                + "; the cells " + cell + "; the pieces " + piece + "; losing " + loss + " per 100 in/s, bouncing"
                + " above " + threshold + " in/s, twisting " + impactTwist + " in on impact and " + spinTwist
                + " in pressed, rolling resistance " + rollingResistance;
    }
}
