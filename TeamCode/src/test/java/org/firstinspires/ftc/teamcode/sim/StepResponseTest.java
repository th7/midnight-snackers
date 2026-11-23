package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import java.util.Optional;
import java.util.OptionalDouble;
import org.firstinspires.ftc.framework.tools.StepResponse;
import org.junit.Test;

public class StepResponseTest {
    private static StepResponse overshootingTo100() {
        StepResponse response = new StepResponse(100);
        response.add(0, 0);
        response.add(1, 80);
        response.add(2, 115);
        response.add(3, 104);
        response.add(4, 99);
        response.add(5, 101);
        return response;
    }

    @Test
    public void itsPeakIsTheFastestSampleAndWhenItCame() {
        StepResponse.Sample peak = overshootingTo100().peak().orElseThrow();

        assertEquals(2, peak.seconds(), 0);
        assertEquals(115, peak.value(), 0);
    }

    @Test
    public void itSettlesAtTheFirstSampleAfterTheLastOneOutsideTheBand() {
        assertEquals(OptionalDouble.of(4), overshootingTo100().settledWithin(0.02));
        assertEquals(OptionalDouble.of(3), overshootingTo100().settledWithin(0.05));
    }

    @Test
    public void aResponseThatEndsOutsideTheBandHasNotSettled() {
        StepResponse response = overshootingTo100();
        response.add(6, 90);

        assertEquals(OptionalDouble.empty(), response.settledWithin(0.02));
    }

    @Test
    public void anEmptyResponseHasNoPeakAndHasNotSettled() {
        StepResponse response = new StepResponse(100);

        assertEquals(Optional.empty(), response.peak());
        assertEquals(OptionalDouble.empty(), response.settledWithin(0.02));
    }

    @Test
    public void aResponseNeverOutsideTheBandSettledAtItsFirstSample() {
        StepResponse response = new StepResponse(100);
        response.add(0.5, 100);
        response.add(1, 101);

        assertEquals(OptionalDouble.of(0.5), response.settledWithin(0.02));
    }
}
