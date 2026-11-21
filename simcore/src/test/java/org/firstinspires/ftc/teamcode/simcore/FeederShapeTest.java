package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FeederShapeTest {
    private static final String CAD_SOURCE = "doc/plans/feeder-sim.md, Phase 1, Tests first, read from the Onshape"
            + " part studio Feeder on 16 November, in mm in the feeder frame";
    private static final double ROUNDING = 1e-9;

    private static final String POLLEN_SOURCE = "doc/plans/feeder-sim.md, What we infer: the plan's numbers are for"
            + " the CAD's 71.12 mm pollen, 1.4 in in radius";
    private static final Length CADS_POLLEN = Valid.length(1.4);
    private static final double TO_THE_HUNDREDTH = 0.01;
    private static final String TABLE_SOURCE = "doc/plans/feeder-sim.md, What we infer, The lift: its table's"
            + " numbers are given to the tenth of a mm. Its lifts are not the CAD's: turned truly about the shaft, the"
            + " CAD's 6 wide bar rounded 1 lifts a centered pollen 12.2 at 30 degrees and 21.0 at 35, where the plan"
            + " says 11.0 and 20.3 (issue 180)";
    private static final double WITHIN_A_TENTH = 0.05;
    private static final String ANGLES_SOURCE = "doc/plans/feeder-sim.md, What the CAD says, The ramp's motion: the"
            + " plan's 27.5 and 17.5 degrees are to the half degree; turned truly about the shaft the CAD gives 27.73"
            + " and 17.29 (issue 180)";
    private static final double WITHIN_A_QUARTER_DEGREE = 0.25;

    private final Feeder.Shape cad = Valid.value(Feeder.Shape.cad());

    @Test
    public void theCadsFeederIsAcceptedWithItsFloorWallAndRoom() {
        assertEquals(15, cad.floorTopMm(), ROUNDING);
        assertEquals(82.27, cad.wallRadiusMm(), ROUNDING);
        assertEquals(81.12, cad.roomMm(), ROUNDING);
    }

    @Test
    public void twoPollenFitOppositeClearingTheHubAndSeatedAgainstTheWall() {
        Feeder.Seat seat = Valid.value(cad.seatedAgainstTheWall(CADS_POLLEN));

        assertEquals(1.15, seat.hubClearanceMm(), TO_THE_HUNDREDTH);
        assertEquals(0, seat.wallClearanceMm(), TO_THE_HUNDREDTH);
        assertEquals("seated midway between the arms", 9.15, seat.armClearanceMm(), TO_THE_HUNDREDTH);
    }

    @Test
    public void aSpinnerWhoseArmsReachR54IsRefusedForTheRaisedBar() {
        Feeder.Spinner reachingFurther = Valid.value(Feeder.Spinner.of(10, 4, 54, 17, 80));

        assertRejected("raised bar", cad.with(reachingFurther));
    }

    @Test
    public void anArmTallerThanTheRoomUnderTheWheelIsRefused() {
        Feeder.Spinner tallerArms = Valid.value(Feeder.Spinner.of(10, 4, 50, 17, 84));

        assertRejected("under the wheel", cad.with(tallerArms));
    }

    @Test
    public void aSpinnerWhoseArmsReachIntoTheFloorIsRefused() {
        Feeder.Spinner intoTheFloor = Valid.value(Feeder.Spinner.of(10, 4, 50, 14, 80));

        assertRejected("above the floor", cad.with(intoTheFloor));
    }

    @Test
    public void aBarWiderThanTheSlotOrReachingPastItsEndIsRefused() {
        Feeder.Bar wider = Valid.value(Feeder.Bar.of(57.54, -13.99, -11.49, 5, 7.5, 1, -53.95, -31.96, 38));
        Feeder.Bar longer = Valid.value(Feeder.Bar.of(57.54, -13.99, -11.49, 5, 6, 1, -53.95, -33, 38));

        assertRejected("slot", cad.with(wider));
        assertRejected("slot", cad.with(longer));
    }

    @Test
    public void theBarsTopIsBelowTheFloorWhenDownAndReachesItAtX0At27Point5DegreesAndAtItsFreeEndAt17Point5() {
        assertTrue(barTop(-31.96, 0) < 15);
        assertTrue(barTop(38, 0) < 15);
        assertTrue(barTop(0, 27.5 - WITHIN_A_QUARTER_DEGREE) < 15);
        assertTrue(barTop(0, 27.5 + WITHIN_A_QUARTER_DEGREE) > 15);
        assertTrue(barTop(-31.96, 17.5 - WITHIN_A_QUARTER_DEGREE) < 15);
        assertTrue(barTop(-31.96, 17.5 + WITHIN_A_QUARTER_DEGREE) > 15);
    }

    @Test
    public void aPollenAtRestAtTheBackStationClearsTheWheelBy11Point9CenteredAnd7Point1AtXPlusOrMinus20() {
        assertEquals(11.9, wheelClearance(0, 0), WITHIN_A_TENTH);
        assertEquals(7.1, wheelClearance(20, 0), WITHIN_A_TENTH);
        assertEquals(7.1, wheelClearance(-20, 0), WITHIN_A_TENTH);
    }

    @Test
    public void liftedByTheBarAt30DegreesACenteredPollenAtTheWallRises12Point2AndAt35Degrees21Point0() {
        assertEquals(12.2, lift(0, 30), WITHIN_A_TENTH);
        assertEquals(21.0, lift(0, 35), WITHIN_A_TENTH);
    }

    @Test
    public void aCenteredPollenLiftedAt30DegreesStillClearsTheWheelAndAt35DegreesMeetsIt() {
        assertEquals(3.1, wheelClearance(0, 30), WITHIN_A_TENTH);
        assertEquals(-3.5, wheelClearance(0, 35), WITHIN_A_TENTH);
    }

    @Test
    public void theTransformPutsTheFloorsTopAtTheAxisAndTheFlywheelsAxleWhereReginaldsCadHasThem() {
        assertAt(new Vec3(0, 169.71, 43.17), cad.inReginaldMm(new Vec3(0, 0, 15)));
        assertAt(new Vec3(0, 217, 180), cad.inReginaldMm(new Vec3(0, -84.13, 132.82)));
    }

    @Test
    public void aWindowWhoseSillIsNotBetweenTheFloorAndTheTopPlateOrWiderThanTheWallIsRefused() {
        Feeder.Window overThePlate = Valid.value(Feeder.Window.of(63, 100));
        Feeder.Window underTheFloor = Valid.value(Feeder.Window.of(40.6, 10));
        Feeder.Window widerThanTheWall = Valid.value(Feeder.Window.of(83, 31.12));

        assertRejected("sill", cad.withFrontWindow(overThePlate));
        assertRejected("sill", cad.withBackWindow(underTheFloor));
        assertRejected("wall", cad.withFrontWindow(widerThanTheWall));
    }

    @Test
    public void aNotchThatDoesNotEndInsideThePlateOrIsWiderThanTheWallIsRefused() {
        Feeder.Plate endingPastTheFront = Valid.value(Feeder.Plate.of(-78.78, 62.28, 31.739, 70));
        Feeder.Plate widerThanTheWall = Valid.value(Feeder.Plate.of(-78.78, 62.28, 83, -26.843));

        assertRejected("notch", cad.with(endingPastTheFront));
        assertRejected("wall", cad.with(widerThanTheWall));
    }

    private static void assertAt(Vec3 expected, Vec3 actual) {
        assertEquals("x of " + actual, expected.x(), actual.x(), TO_THE_HUNDREDTH);
        assertEquals("y of " + actual, expected.y(), actual.y(), TO_THE_HUNDREDTH);
        assertEquals("z of " + actual, expected.z(), actual.z(), TO_THE_HUNDREDTH);
    }

    private double lift(double xMm, double raisedDegrees) {
        return Valid.value(cad.atTheBackStation(CADS_POLLEN, xMm)
                .then(station -> Feeder.RampAngle.ofDegrees(raisedDegrees).map(station::liftMm)));
    }

    private double wheelClearance(double xMm, double raisedDegrees) {
        return Valid.value(cad.atTheBackStation(CADS_POLLEN, xMm)
                .then(station -> Feeder.RampAngle.ofDegrees(raisedDegrees).map(station::wheelClearanceMm)));
    }

    private double barTop(double drawnXMm, double raisedDegrees) {
        return Valid.value(Feeder.RampAngle.ofDegrees(raisedDegrees).then(raised -> cad.barTopMm(drawnXMm, raised)));
    }

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }
}
