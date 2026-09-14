package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class LoopGroupTest {
    private final List<String> ticks = new ArrayList<>();
    private final LoopGroup group = new LoopGroup();

    private Loopable named(String name) {
        return () -> ticks.add(name);
    }

    @Test
    public void ticksMembersInTheOrderTheyWereAdded() {
        group.add(named("first"));
        group.add(named("second"));
        group.add(named("third"));

        group.loop();

        assertEquals(List.of("first", "second", "third"), ticks);
    }

    @Test
    public void ticksEveryMemberOnEveryLoop() {
        group.add(named("a"));
        group.add(named("b"));

        group.loop();
        group.loop();

        assertEquals(List.of("a", "b", "a", "b"), ticks);
    }

    @Test
    public void addReturnsTheMemberSoItCanBeAssignedInline() {
        Loopable member = named("only");

        assertSame(member, group.add(member));
    }

    @Test
    public void aMemberAddedTwiceTicksTwice() {
        Loopable member = named("dup");
        group.add(member);
        group.add(member);

        group.loop();

        assertEquals(List.of("dup", "dup"), ticks);
    }

    @Test
    public void membersReportsTheLoopOrder() {
        Loopable a = named("a");
        Loopable b = named("b");
        group.add(a);
        group.add(b);

        assertEquals(List.of(a, b), group.members());
    }
}
