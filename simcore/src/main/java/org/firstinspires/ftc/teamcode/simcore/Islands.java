package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

final class Islands {
    enum State {
        MOVING,
        RESTING,
        ASLEEP,
        HELD
    }

    record Node(String name, State state, Vec3 at, double radius, Set<String> touching) {}

    private Islands() {}

    static Set<String> toWake(List<Node> nodes, double within) {
        Set<String> reached = new HashSet<>();
        Set<String> woken = new LinkedHashSet<>();
        List<Node> frontier = new ArrayList<>();
        for (Node node : nodes) {
            if (node.state() == State.MOVING) {
                frontier.add(node);
                reached.add(node.name());
            }
        }
        while (!frontier.isEmpty()) {
            List<Node> next = new ArrayList<>();
            for (Node from : frontier) {
                for (Node to : nodes) {
                    if (to.state() == State.HELD || reached.contains(to.name())) {
                        continue;
                    }
                    boolean heldUp =
                            from.state() == State.MOVING && from.touching().contains(to.name());
                    if (heldUp || touching(from, to, within)) {
                        reached.add(to.name());
                        next.add(to);
                        if (to.state() == State.ASLEEP) {
                            woken.add(to.name());
                        }
                    }
                }
            }
            frontier = next;
        }
        return woken;
    }

    private static boolean touching(Node one, Node other, double within) {
        Vec3 apart = one.at().minus(other.at());
        double reach = one.radius() + other.radius() + within;
        return apart.dot(apart) <= reach * reach;
    }
}
