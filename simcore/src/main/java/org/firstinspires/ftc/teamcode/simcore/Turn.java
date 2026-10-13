package org.firstinspires.ftc.teamcode.simcore;

public final class Turn {
    private final double w;
    private final double x;
    private final double y;
    private final double z;

    private Turn(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Turn identity() {
        return new Turn(1, 0, 0, 0);
    }

    public Turn turnedBy(Vec3 spin, Seconds over) {
        double rate = StrictMath.sqrt(spin.dot(spin));
        double angle = rate * over.value();
        if (!(angle > 0) || !Double.isFinite(angle)) {
            return this;
        }
        double half = angle / 2;
        double along = StrictMath.sin(half) / rate;
        double dw = StrictMath.cos(half), dx = spin.x() * along, dy = spin.y() * along, dz = spin.z() * along;
        double nw = dw * w - dx * x - dy * y - dz * z;
        double nx = dw * x + dx * w + dy * z - dz * y;
        double ny = dw * y - dx * z + dy * w + dz * x;
        double nz = dw * z + dx * y - dy * x + dz * w;
        double norm = StrictMath.sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
        return new Turn(nw / norm, nx / norm, ny / norm, nz / norm);
    }

    public double w() {
        return w;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double z() {
        return z;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Turn that
                && Double.compare(w, that.w) == 0
                && Double.compare(x, that.x) == 0
                && Double.compare(y, that.y) == 0
                && Double.compare(z, that.z) == 0;
    }

    @Override
    public int hashCode() {
        return ((Double.hashCode(w) * 31 + Double.hashCode(x)) * 31 + Double.hashCode(y)) * 31 + Double.hashCode(z);
    }

    @Override
    public String toString() {
        return "turned (" + w + ", " + x + ", " + y + ", " + z + ")";
    }
}
