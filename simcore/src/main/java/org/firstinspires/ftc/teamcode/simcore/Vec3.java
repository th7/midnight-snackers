package org.firstinspires.ftc.teamcode.simcore;

public record Vec3(double x, double y, double z) {
    public static Vec3 zero() {
        return new Vec3(0, 0, 0);
    }

    public Vec3 plus(Vec3 other) {
        return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    public Vec3 minus(Vec3 other) {
        return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    public Vec3 times(double by) {
        return new Vec3(x * by, y * by, z * by);
    }

    public Vec3 negated() {
        return new Vec3(-x, -y, -z);
    }

    public Vec3 along(Vec3 direction, double distance) {
        return new Vec3(x + direction.x * distance, y + direction.y * distance, z + direction.z * distance);
    }

    public double dot(Vec3 other) {
        return x * other.x + y * other.y + z * other.z;
    }

    public Vec3 cross(Vec3 other) {
        return new Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    public double length() {
        return Math.sqrt(dot(this));
    }

    public Vec3 unit() {
        return times(1 / length());
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    public static Vec3 newellNormalOf(Ring<Vec3> ring) {
        double nx = 0, ny = 0, nz = 0;
        for (Ring.Edge<Vec3> edge : ring.edges()) {
            Vec3 a = edge.from(), b = edge.to();
            nx += (a.y - b.y) * (a.z + b.z);
            ny += (a.z - b.z) * (a.x + b.x);
            nz += (a.x - b.x) * (a.y + b.y);
        }
        return new Vec3(nx, ny, nz);
    }

    public static Checked<Vec3> unitNormalOf(Ring<Vec3> ring) {
        Vec3 n = newellNormalOf(ring);
        double length = Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (!(length > 0)) {
            return Checked.rejected("a face has an area, and " + ring + " has none");
        }
        return Checked.ok(new Vec3(n.x / length, n.y / length, n.z / length));
    }
}
