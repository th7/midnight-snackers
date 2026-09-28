package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.junit.Test;

public class ValidTest {
    @Test
    public void aValueThatPassedItsRuleIsTheValue() {
        assertEquals("ok", Valid.value(Checked.ok("ok")));
    }

    @Test
    public void aValueThatBrokeItsRuleIsRefusedNamingTheRule() {
        try {
            Valid.value(Checked.rejected("a length is positive"));
            fail();
        } catch (IllegalArgumentException e) {
            assertEquals("a length is positive", e.getMessage());
        }
    }
}
