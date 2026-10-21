package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DeploymentTest {
    @Test
    public void onlyAZeroExitSaysTheRobotWasDeployed() {
        Deployment installed = Deployment.exited(0, "installed");
        assertTrue(installed.ok());
        assertEquals(200, installed.status());
        assertEquals(Deployment.Outcome.INSTALLED, installed.outcome());
        assertTrue(installed.message(), installed.message().contains("deployed"));
        assertEquals("installed", installed.output());
        for (int exit : new int[] {1, 127, -1, Integer.MIN_VALUE, Integer.MAX_VALUE}) {
            Deployment failed = Deployment.exited(exit, "compile or adb failed");
            assertFalse(failed.ok());
            assertEquals(500, failed.status());
            assertEquals(Deployment.Outcome.FAILED, failed.outcome());
            assertEquals("compile or adb failed", failed.output());
        }
    }

    @Test
    public void interruptedTimedOutAndUnavailableDeploymentsNeverSayTheySucceeded() {
        for (Deployment result : new Deployment[] {
            Deployment.timedOut("the diagnostic"),
            Deployment.interrupted("the diagnostic"),
            Deployment.couldNotStart("the diagnostic")
        }) {
            assertFalse(result.ok());
            assertEquals("the diagnostic", result.output());
            assertEquals(500, result.status());
            assertFalse(result.message(), result.message().isBlank());
        }
        assertEquals(409, Deployment.busy().status());
        assertFalse(Deployment.busy().ok());
    }
}
