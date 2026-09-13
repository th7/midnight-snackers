package org.firstinspires.ftc.teamcode.base;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Ticks its members every loop, in the order they were added. */
public class LoopGroup implements Loopable {
    private final List<Loopable> members = new ArrayList<>();

    /** Registers and returns the member so it can be assigned on the same line. */
    public <T extends Loopable> T add(T member) {
        members.add(member);
        return member;
    }

    /** The members in the order they are ticked. */
    public List<Loopable> members() {
        return Collections.unmodifiableList(members);
    }

    @Override
    public void loop() {
        for (Loopable member : members) {
            member.loop();
        }
    }
}
