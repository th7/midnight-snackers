package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Vec2;
import org.firstinspires.ftc.teamcode.simcore.Vec3;

public final class Points {
    private Points() {}

    public static double[] array(Vec3 v) {
        return new double[] {v.x(), v.y(), v.z()};
    }

    public static Vec3 vec(double[] a) {
        return new Vec3(a[0], a[1], a[2]);
    }

    public static double[][] arrays(Ring<Vec3> ring) {
        List<Vec3> all = ring.all();
        double[][] out = new double[all.size()][];
        for (int i = 0; i < out.length; i++) {
            out[i] = array(all.get(i));
        }
        return out;
    }

    public static List<double[][]> arrays(List<Ring<Vec3>> rings) {
        List<double[][]> out = new ArrayList<>();
        for (Ring<Vec3> ring : rings) {
            out.add(arrays(ring));
        }
        return out;
    }

    public static double[][] footprint(Field.Obstacle obstacle) {
        List<Vec2> corners = obstacle.footprint().corners();
        double[][] out = new double[corners.size()][];
        for (int i = 0; i < out.length; i++) {
            out[i] = new double[] {corners.get(i).x(), corners.get(i).y()};
        }
        return out;
    }

    public static double[] axis(Field.Flower flower) {
        return new double[] {flower.axis().x(), flower.axis().y()};
    }

    public static double[] normal(double[][] ring) {
        double[] n = new double[3];
        for (int i = 0; i < ring.length; i++) {
            double[] a = ring[i], b = ring[(i + 1) % ring.length];
            n[0] += (a[1] - b[1]) * (a[2] + b[2]);
            n[1] += (a[2] - b[2]) * (a[0] + b[0]);
            n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        double length = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        return new double[] {n[0] / length, n[1] / length, n[2] / length};
    }
}
