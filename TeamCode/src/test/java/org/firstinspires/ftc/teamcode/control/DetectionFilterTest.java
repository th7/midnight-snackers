package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.OptionalLong;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.junit.Test;

/**
 * The filter answers one question -- have three consistent, recent detections agreed? -- and every
 * way of asking it is total: a filter that has seen nothing answers, rather than throwing.
 */
public class DetectionFilterTest {
    private static final long TENTH_OF_A_SECOND = 100_000_000L;

    private long now = 0;
    private final DetectionFilter filter = new DetectionFilter(() -> now);

    /**
     * The one a driver reaches on the field: telemetry asks what the filter has seen before any tag
     * has been seen at all.
     */
    @Test
    public void aFilterThatHasSeenNothingAgreesOnNothingAndHasNoAge() {
        assertTrue("no agreement", filter.agreed().isEmpty());
        assertEquals("no age to report", OptionalLong.empty(), filter.lastAgeNano());
    }

    @Test
    public void threeConsistentRecentDetectionsAgreeOnTheNewestOfThem() {
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10.1, 20.1));
        AprilTagDetection newest = detection(10.2, 20.2);
        filter.addDetection(newest);

        DetectionFilter.Agreed agreed = filter.agreed().orElseThrow();

        assertEquals(newest, agreed.detection());
        assertEquals(0, agreed.ageNano());
    }

    @Test
    public void twoDetectionsAreNotEnoughToAgree() {
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));

        assertTrue(filter.agreed().isEmpty());
    }

    @Test
    public void detectionsThatDisagreeOnPositionDoNotAgree() {
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(40, 20));

        assertTrue(filter.agreed().isEmpty());
    }

    /** An agreement goes stale a tenth of a second after its frame, on the robot's clock. */
    @Test
    public void anAgreementGoesStaleATenthOfASecondAfterItsNewestFrame() {
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));

        now = TENTH_OF_A_SECOND - 1;
        assertTrue("still fresh", filter.agreed().isPresent());
        assertEquals(TENTH_OF_A_SECOND - 1, filter.agreed().orElseThrow().ageNano());

        now = TENTH_OF_A_SECOND;
        assertTrue("stale", filter.agreed().isEmpty());
    }

    /**
     * The age is the newest detection's, agreement or not: it is what a driver watching telemetry
     * needs exactly when nothing is agreeing.
     */
    @Test
    public void theAgeIsReadableWhileNothingAgrees() {
        filter.addDetection(detection(10, 20));
        now = 5_000_000L;

        assertTrue("one detection cannot agree", filter.agreed().isEmpty());
        assertEquals(OptionalLong.of(5_000_000L), filter.lastAgeNano());
    }

    /** Only the last three count, so an old outlier stops holding agreement back. */
    @Test
    public void anOutlierIsForgottenOnceThreeNewerDetectionsHaveArrived() {
        filter.addDetection(detection(40, 20));
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));
        assertTrue("the outlier is still one of the three", filter.agreed().isEmpty());

        filter.addDetection(detection(10, 20));

        assertTrue("the outlier has fallen off the end", filter.agreed().isPresent());
    }

    @Test
    public void aDetectionWithNoRobotPoseCannotAgree() {
        filter.addDetection(detection(10, 20));
        filter.addDetection(detection(10, 20));
        filter.addDetection(new AprilTagDetection(20, 0, 0, null, null, null, null, null, null, now));

        assertTrue(filter.agreed().isEmpty());
        assertFalse("but its age is still readable", filter.lastAgeNano().isEmpty());
    }

    private AprilTagDetection detection(double x, double y) {
        Pose3D robotPose = new Pose3D(
                new Position(DistanceUnit.INCH, x, y, 0, now), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, now));
        return new AprilTagDetection(20, 0, 0, null, null, null, null, null, robotPose, now);
    }
}
