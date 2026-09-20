package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class SimHives {
    public static final int FULL = 40;
    public static final int NECTAR_FILLS = FULL / 5;
    public static final int POLLEN_FILLS = FULL / 8;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    public static final class Occupant {
        public final Object ball;
        public final String kind;
        public final double radius;

        Occupant(Object ball, String kind, double radius) {
            this.ball = ball;
            this.kind = kind;
            this.radius = radius;
        }
    }

    public static final class Met {
        public static final Met NOTHING = new Met(false, null, null, null);
        static final Met MOUTH_FROM_BEHIND = new Met(true, null, null, null);

        private final boolean met;
        public final SimField.Cell scoredIn;
        public final double[] at;
        public final double[] velocity;

        private Met(boolean met, SimField.Cell scoredIn, double[] at, double[] velocity) {
            this.met = met;
            this.scoredIn = scoredIn;
            this.at = at;
            this.velocity = velocity;
        }

        static Met scored(SimField.Cell cell) {
            return new Met(true, cell, null, null);
        }

        static Met bounced(double[] at, double[] velocity) {
            return new Met(true, null, at, velocity);
        }

        public boolean met() {
            return met;
        }

        public boolean scored() {
            return scoredIn != null;
        }

        public boolean bounced() {
            return at != null;
        }
    }

    public static final class LeftACell {
        public final Object ball;
        public final double[] at;
        public final double[] velocity;

        LeftACell(Object ball, double[] at, double[] velocity) {
            this.ball = ball;
            this.at = at;
            this.velocity = velocity;
        }
    }

    private static final class Turned {
        final double[][] mouth;
        final double[] mouthNormal;
        final List<double[][]> panels;
        final boolean upturned;
        final double[] floorAtTheBack;
        final double[] towardTheMouth;
        final double[] acrossTheFloor;
        final double[] offTheFloor;
        final double floorWidth;

        Turned(SimField.Cell cell, double tilt) {
            this.mouth = cell.mouthAt(tilt);
            this.mouthNormal = cell.mouthNormalAt(tilt);
            this.panels = cell.panelsAt(tilt);
            this.upturned = cell.upturnedAt(tilt);
            double[][] floor = floorOf(cell.back);
            this.floorAtTheBack = cell.hive.at(tilt, mean(floor));
            this.floorWidth = floor[floor.length - 1][1] - floor[0][1];
            this.towardTheMouth = cell.hive.direction(tilt, new double[] {Math.signum(cell.mouthNormal[0]), 0, 0});
            this.acrossTheFloor = cell.hive.direction(tilt, new double[] {0, 1, 0});
            this.offTheFloor = cell.hive.direction(tilt, new double[] {0, 0, 1});
        }

        private static double[][] floorOf(double[][] ring) {
            double lowest = Double.MAX_VALUE;
            for (double[] corner : ring) {
                lowest = Math.min(lowest, corner[2]);
            }
            List<double[]> floor = new ArrayList<>();
            for (double[] corner : ring) {
                if (corner[2] <= lowest + 0.2) {
                    floor.add(corner);
                }
            }
            floor.sort((a, b) -> Double.compare(a[1], b[1]));
            return floor.toArray(new double[0][]);
        }

        private static double[] mean(double[][] ring) {
            double[] out = new double[3];
            for (double[] corner : ring) {
                for (int axis = 0; axis < 3; axis++) {
                    out[axis] += corner[axis] / ring.length;
                }
            }
            return out;
        }
    }

    private final SimField field;
    private final double bounce;
    private final double rollOutInchesPerSecond;
    private final Map<SimField.Hive, Double> tilts = new LinkedHashMap<>();
    private final Map<SimField.Cell, Turned> turned = new LinkedHashMap<>();
    private final Map<SimField.Cell, List<Occupant>> inCell = new LinkedHashMap<>();
    private final Map<Object, SimField.Cell> cellOf = new IdentityHashMap<>();

    public SimHives(SimField field, double bounce, double rollOutInchesPerSecond) {
        this.field = field;
        this.bounce = bounce;
        this.rollOutInchesPerSecond = rollOutInchesPerSecond;
        for (SimField.Hive hive : field.hives) {
            tilts.put(hive, hive.tilt);
        }
        for (SimField.Cell cell : field.cells) {
            inCell.put(cell, new ArrayList<>());
            turned.put(cell, new Turned(cell, tilts.get(cell.hive)));
        }
    }

    public SimField.Hive hiveOf(String alliance) {
        for (SimField.Hive hive : field.hives) {
            if (hive.alliance.equals(alliance)) {
                return hive;
            }
        }
        throw new IllegalArgumentException("no hive for " + alliance);
    }

    public double tilt(String alliance) {
        return tilts.get(hiveOf(alliance));
    }

    public Map<String, Double> tilts() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (SimField.Hive hive : field.hives) {
            out.put(hive.alliance, tilts.get(hive));
        }
        return out;
    }

    public SimField.Cell upturnedCell(String alliance) {
        for (SimField.Cell cell : hiveOf(alliance).cells) {
            if (turned.get(cell).upturned) {
                return cell;
            }
        }
        throw new IllegalStateException(alliance + "'s hive has no upturned cell");
    }

    public boolean upturned(SimField.Cell cell) {
        return turned.get(cell).upturned;
    }

    public int scored(String alliance) {
        int total = 0;
        for (Map.Entry<SimField.Cell, List<Occupant>> entry : inCell.entrySet()) {
            if (entry.getKey().alliance.equals(alliance)) {
                total += entry.getValue().size();
            }
        }
        return total;
    }

    public Map<String, Integer> scored() {
        Map<String, Integer> out = new LinkedHashMap<>();
        for (SimField.Cell cell : inCell.keySet()) {
            out.merge(cell.alliance, inCell.get(cell).size(), Integer::sum);
        }
        return out;
    }

    public double load(String alliance) {
        return fill(hiveOf(alliance)) / (double) FULL;
    }

    public int fill(SimField.Hive hive) {
        int fill = 0;
        for (SimField.Cell cell : hive.cells) {
            for (Occupant occupant : inCell.get(cell)) {
                fill += SimField.NECTAR.equals(occupant.kind) ? NECTAR_FILLS : POLLEN_FILLS;
            }
        }
        return fill;
    }

    public void put(Object ball, String kind, double radius, SimField.Cell cell) {
        takeOut(ball);
        inCell.get(cell).add(new Occupant(ball, kind, radius));
        cellOf.put(ball, cell);
    }

    public void takeOut(Object ball) {
        SimField.Cell was = cellOf.remove(ball);
        if (was != null) {
            inCell.get(was).removeIf(occupant -> occupant.ball == ball);
        }
    }

    public boolean holds(Object ball) {
        return cellOf.containsKey(ball);
    }

    public double[] restingPlace(Object ball) {
        SimField.Cell cell = cellOf.get(ball);
        if (cell == null) {
            throw new IllegalArgumentException("no cell holds that ball");
        }
        Turned turn = turned.get(cell);
        List<Occupant> occupants = inCell.get(cell);
        int slot = -1;
        for (int i = 0; i < occupants.size(); i++) {
            if (occupants.get(i).ball == ball) {
                slot = i;
            }
        }
        double radius = occupants.get(slot).radius;
        int perRow = Math.max(1, (int) (turn.floorWidth / (2 * radius)));
        double across = (slot % perRow - (perRow - 1) / 2.0) * 2 * radius;
        double along = radius + slot / perRow * 2 * radius;
        double[] out = new double[3];
        for (int axis = 0; axis < 3; axis++) {
            out[axis] = turn.floorAtTheBack[axis]
                    + turn.towardTheMouth[axis] * along
                    + turn.acrossTheFloor[axis] * across
                    + turn.offTheFloor[axis] * radius;
        }
        return out;
    }

    public Met met(double[] from, double[] to, double[] velocity) {
        for (SimField.Cell cell : field.cells) {
            Turned turn = turned.get(cell);
            if (crossing(turn.mouth, turn.mouthNormal, from, to) != null) {
                return side(turn.mouthNormal, turn.mouth[0], from) > 0 ? Met.scored(cell) : Met.MOUTH_FROM_BEHIND;
            }
            for (double[][] panel : turn.panels) {
                double[] normal = SimField.normal(panel);
                double[] hit = crossing(panel, normal, from, to);
                if (hit == null) {
                    continue;
                }
                boolean fromTheFront = side(normal, panel[0], from) > 0;
                double along = velocity[0] * normal[0] + velocity[1] * normal[1] + velocity[2] * normal[2];
                double back = fromTheFront ? CONTACT_TOLERANCE_IN : -CONTACT_TOLERANCE_IN;
                return Met.bounced(
                        new double[] {hit[0] + back * normal[0], hit[1] + back * normal[1], hit[2] + back * normal[2]},
                        new double[] {
                            velocity[0] - (1 + bounce) * along * normal[0],
                            velocity[1] - (1 + bounce) * along * normal[1],
                            velocity[2] - (1 + bounce) * along * normal[2]
                        });
            }
        }
        return Met.NOTHING;
    }

    public List<LeftACell> turn() {
        List<LeftACell> left = new ArrayList<>();
        for (SimField.Hive hive : field.hives) {
            if (fill(hive) >= FULL) {
                tilts.put(hive, -tilts.get(hive));
                for (SimField.Cell cell : hive.cells) {
                    turned.put(cell, new Turned(cell, tilts.get(hive)));
                }
            }
            for (SimField.Cell cell : hive.cells) {
                if (turned.get(cell).upturned) {
                    continue;
                }
                Turned turn = turned.get(cell);
                for (Occupant occupant : new ArrayList<>(inCell.get(cell))) {
                    double[] out = restingPlace(occupant.ball);
                    takeOut(occupant.ball);
                    left.add(new LeftACell(
                            occupant.ball,
                            new double[] {
                                out[0] + turn.mouthNormal[0] * CONTACT_TOLERANCE_IN,
                                out[1] + turn.mouthNormal[1] * CONTACT_TOLERANCE_IN,
                                out[2] + turn.mouthNormal[2] * CONTACT_TOLERANCE_IN
                            },
                            new double[] {
                                turn.mouthNormal[0] * rollOutInchesPerSecond,
                                turn.mouthNormal[1] * rollOutInchesPerSecond,
                                turn.mouthNormal[2] * rollOutInchesPerSecond
                            }));
                }
            }
        }
        return left;
    }

    static double side(double[] normal, double[] point, double[] position) {
        return (position[0] - point[0]) * normal[0]
                + (position[1] - point[1]) * normal[1]
                + (position[2] - point[2]) * normal[2];
    }

    static double[] crossing(double[][] panel, double[] normal, double[] from, double[] to) {
        double before = side(normal, panel[0], from);
        double after = side(normal, panel[0], to);
        if (before == 0 || (before > 0) == (after > 0)) {
            return null;
        }
        double t = before / (before - after);
        double[] hit = {
            from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t, from[2] + (to[2] - from[2]) * t
        };

        int drop = 0;
        for (int axis = 1; axis < 3; axis++) {
            if (Math.abs(normal[axis]) > Math.abs(normal[drop])) {
                drop = axis;
            }
        }
        int u = (drop + 1) % 3, v = (drop + 2) % 3;
        boolean inside = false;
        for (int i = 0, j = panel.length - 1; i < panel.length; j = i++) {
            double[] a = panel[i], b = panel[j];
            if ((a[v] > hit[v]) != (b[v] > hit[v]) && hit[u] < (b[u] - a[u]) * (hit[v] - a[v]) / (b[v] - a[v]) + a[u]) {
                inside = !inside;
            }
        }
        return inside ? hit : null;
    }
}
