package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

public final class FieldGlb {
    public static final double GRID_IN = 0.05;
    public static final double NO_GRID = 0;
    public static final int LEAST_COMPARED = 10;

    private static final Pattern SKIP = Pattern.compile(
            "screw|fhts|nut\\b|washer|rivet|rivnut|bolt|spacer|bearing|cable tie|plug|\\bpin\\b|"
                    + "damper|hinge|panel link|strap|clip|under tile|peanut|"
                    + "fastener|quick release|soft tiles|perimeter|rail|side glass",
            Pattern.CASE_INSENSITIVE);

    private static final Pattern KEEP = Pattern.compile("field panel|ftc rail|side glass", Pattern.CASE_INSENSITIVE);

    private static final Pattern BLUE = Pattern.compile("blue", Pattern.CASE_INSENSITIVE);
    private static final Pattern RED = Pattern.compile("red", Pattern.CASE_INSENSITIVE);

    private FieldGlb() {}

    public static byte[] build(byte[] export, double fieldSizeIn, double grid) {
        List<Gltf.Part> parts = visualParts(Gltf.read(export), grid);
        if (parts.isEmpty()) {
            throw new IllegalStateException("nothing was kept; the export named none of the parts we draw");
        }
        List<String> off = disagreements(parts, fieldSizeIn);
        if (!off.isEmpty()) {
            throw new IllegalStateException(
                    "the visual model and the collision model disagree about where the field is:\n  "
                            + String.join("\n  ", off));
        }
        return Gltf.write(parts);
    }

    public static List<Gltf.Part> visualParts(List<Gltf.Part> parts, double grid) {
        List<Gltf.Part> out = new ArrayList<>();
        for (Gltf.Part part : parts) {
            if (!KEEP.matcher(part.name).find() && droppedByName(part)) {
                continue;
            }
            double[] snapped = snap(toField(part.triangles), grid);
            if (snapped.length > 0) {
                out.add(new Gltf.Part(part.name, part.path, snapped, part.colour));
            }
        }
        return out;
    }

    private static boolean droppedByName(Gltf.Part part) {
        if (SKIP.matcher(part.name).find()) {
            return true;
        }
        for (String above : part.path) {
            if (SKIP.matcher(above).find()) {
                return true;
            }
        }
        return false;
    }

    public static double[] toField(double[] points) {
        double[] out = new double[points.length];
        for (int i = 0; i < points.length; i += 3) {
            out[i] = points[i + 1] / Onshape.INCH_IN_METRES;
            out[i + 1] = -points[i] / Onshape.INCH_IN_METRES;
            out[i + 2] = points[i + 2] / Onshape.INCH_IN_METRES;
        }
        return out;
    }

    public static double[] snap(double[] triangles, double grid) {
        double[] kept = new double[triangles.length];
        int at = 0;
        for (int i = 0; i < triangles.length; i += 9) {
            double[] corner = new double[9];
            for (int c = 0; c < 9; c++) {
                corner[c] = grid > 0 ? Math.rint(triangles[i + c] / grid) * grid : triangles[i + c];
            }
            if (same(corner, 0, 3) || same(corner, 3, 6) || same(corner, 0, 6)) {
                continue;
            }
            System.arraycopy(corner, 0, kept, at, 9);
            at += 9;
        }
        double[] out = new double[at];
        System.arraycopy(kept, 0, out, 0, at);
        return out;
    }

    private static boolean same(double[] corner, int a, int b) {
        return corner[a] == corner[b] && corner[a + 1] == corner[b + 1] && corner[a + 2] == corner[b + 2];
    }

    public static List<String> disagreements(List<Gltf.Part> parts, double fieldSizeIn) {
        List<double[]> blue = new ArrayList<>();
        List<double[]> red = new ArrayList<>();
        for (Gltf.Part part : parts) {
            if (BLUE.matcher(part.name).find()) {
                blue.add(middleOf(part));
            }
            if (RED.matcher(part.name).find()) {
                red.add(middleOf(part));
            }
        }

        List<String> off = new ArrayList<>();
        if (blue.size() < LEAST_COMPARED || red.size() < LEAST_COMPARED) {
            off.add("could not judge which way round the field is: " + blue.size() + " blue parts and " + red.size()
                    + " red, which is fewer than the " + LEAST_COMPARED + " it takes to tell");
            return off;
        }

        long blueWrong = blue.stream().filter(centre -> centre[1] >= 0).count();
        long redWrong = red.stream().filter(centre -> centre[1] <= 0).count();
        if (blueWrong > 0 || redWrong > 0) {
            off.add(String.format(
                    Locale.ROOT,
                    "the field is the wrong way round: blue belongs at negative y and red at positive, and %d of %d "
                            + "blue parts and %d of %d red are on the wrong side",
                    blueWrong,
                    blue.size(),
                    redWrong,
                    red.size()));
        }

        double half = fieldSizeIn / 2;
        double reach = 0;
        for (Gltf.Part part : parts) {
            for (int i = 0; i < part.triangles.length; i += 3) {
                reach = Math.max(reach, Math.abs(part.triangles[i]));
                reach = Math.max(reach, Math.abs(part.triangles[i + 1]));
            }
        }
        if (!(half * 0.9 < reach && reach < half * 3)) {
            off.add(String.format(
                    Locale.ROOT,
                    "the model reaches %.1f in from the middle; the collision model puts the walls at %.1f in, so "
                            + "the frame or the units are wrong",
                    reach,
                    half));
        }
        return off;
    }

    private static double[] middleOf(Gltf.Part part) {
        double[] sum = new double[3];
        int points = part.triangles.length / 3;
        for (int i = 0; i < part.triangles.length; i += 3) {
            sum[0] += part.triangles[i];
            sum[1] += part.triangles[i + 1];
            sum[2] += part.triangles[i + 2];
        }
        return new double[] {sum[0] / points, sum[1] / points, sum[2] / points};
    }
}
