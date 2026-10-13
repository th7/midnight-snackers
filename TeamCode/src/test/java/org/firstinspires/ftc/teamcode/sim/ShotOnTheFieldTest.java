package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.EnumMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Colliders;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting;
import org.firstinspires.ftc.teamcode.simcore.ShotSettings;
import org.firstinspires.ftc.teamcode.simcore.Surface;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class ShotOnTheFieldTest {
    private static final Field FIELD = SimPlacement.FIELD;
    private static final String BLUE_SCORING = "Blue Cell (Scoring) <1>";
    private static final String BLUE_AUDIENCE = "Blue Cell (Audience) <1>";

    private static ShotSettings with(Object... pairs) {
        Map<ShotSetting, Double> set = new EnumMap<>(ShotSetting.class);
        for (int i = 0; i < pairs.length; i += 2) {
            set.put((ShotSetting) pairs[i], ((Number) pairs[i + 1]).doubleValue());
        }
        return Valid.value(ShotSettings.of(set));
    }

    private static Shot.Finished flown(ShotSettings settings) {
        Shot shot = Valid.value(Shot.from(FIELD, settings)).advance(Valid.value(Seconds.of(25)));
        assertTrue("the shot finished", shot instanceof Shot.Finished);
        return (Shot.Finished) shot;
    }

    @Test
    public void theDefaultLaunchScoresInTheBlueHivesUpturnedCellClearOfTheNectarThere() {
        Shot.Finished shot = flown(ShotSettings.defaults());

        assertTrue(shot.finish().toString(), shot.finish() instanceof Shot.Finish.AtRest);
        Shot.Finish.AtRest rest = (Shot.Finish.AtRest) shot.finish();
        assertEquals(new Shot.Resting.InCell(BLUE_SCORING, "Blue", true), rest.where());
        for (Shot.Track track : shot.story().pieces()) {
            Shot.Placed last = track.samples().get(track.samples().size() - 1);
            double apart = last.at().minus(rest.at()).length();
            assertTrue(
                    track.piece().name() + " is " + apart + " in from the pollen",
                    apart
                            >= track.piece().shape().radius()
                                    + shot.story().pollen().radius()
                                    - 1e-3);
        }
        assertTrue(shot.story().crossings().stream()
                .anyMatch(crossed -> crossed.crossing().cell().equals(BLUE_SCORING)
                        && crossed.crossing().into()));
    }

    @Test
    public void aPollenThatClearsAWallLeavesTheFieldOverThatWall() {
        Shot.Finished shot = flown(with(
                ShotSetting.X,
                0,
                ShotSetting.Y,
                50,
                ShotSetting.YAW,
                90,
                ShotSetting.PITCH,
                35,
                ShotSetting.WHEEL_ONE_RPM,
                2500,
                ShotSetting.WHEEL_TWO_RPM,
                2500));

        assertTrue(shot.finish().toString(), shot.finish() instanceof Shot.Finish.LeftTheField);
        assertEquals(Colliders.Side.RED, ((Shot.Finish.LeftTheField) shot.finish()).side());
    }

    @Test
    public void aPollenFiredAtTheUndersideOfACellAsHardAsTheWheelsGoNeverGetsInsideIt() {
        Shot.Finished shot = flown(with(
                ShotSetting.X,
                14.1,
                ShotSetting.Y,
                -12.76,
                ShotSetting.HEIGHT,
                20,
                ShotSetting.PITCH,
                90,
                ShotSetting.WHEEL_ONE_DIAMETER,
                6,
                ShotSetting.WHEEL_TWO_DIAMETER,
                6,
                ShotSetting.WHEEL_ONE_RPM,
                6000,
                ShotSetting.WHEEL_TWO_RPM,
                6000));

        Colliders colliders = shot.story().world().colliders();
        for (Shot.Frame frame : shot.story().frames()) {
            assertFalse(
                    "inside a cell at " + frame.t(),
                    colliders.cellHolding(frame.pollen().at()).isPresent());
        }
        assertFalse(shot.story().impacts().isEmpty());
    }

    @Test
    public void sidespinThrowsABallSidewaysOffAWall() {
        double plain = sidewaysOffTheFarWall(0), left = sidewaysOffTheFarWall(90), right = sidewaysOffTheFarWall(-90);

        assertEquals(0, plain, 1);
        assertTrue("left " + left + ", right " + right, left * right < 0);
        assertTrue("left " + left, Math.abs(left) > 10);
        assertTrue("right " + right, Math.abs(right) > 10);
    }

    private static double sidewaysOffTheFarWall(double roll) {
        Shot.Finished shot = flown(with(
                ShotSetting.X,
                30,
                ShotSetting.Y,
                40,
                ShotSetting.HEIGHT,
                6,
                ShotSetting.YAW,
                0,
                ShotSetting.PITCH,
                0,
                ShotSetting.ROLL,
                roll,
                ShotSetting.WHEEL_ONE_RPM,
                2400,
                ShotSetting.WHEEL_TWO_RPM,
                1200,
                ShotSetting.PIECES,
                0));
        for (Shot.Impact impact : shot.story().impacts()) {
            if (impact.hit().surface() == Surface.WALL) {
                return impact.velocityAfter().y() - impact.velocityBefore().y();
            }
        }
        throw new AssertionError(
                "the pollen never reached the far wall: " + shot.story().impacts());
    }

    @Test
    public void aPollenLaunchedAtALooseOneWakesItAndPushesItAlongTheWallItIsPinnedTo() {
        Field.Piece target = FIELD.loosePieces().get(0);
        Vec3 toward = new Vec3(Math.signum(target.at().x()), 0, 0);
        Vec3 from = target.at().along(toward, -30).plus(new Vec3(0, -2, 0));
        Shot.Finished shot = flown(with(
                ShotSetting.X,
                from.x(),
                ShotSetting.Y,
                from.y(),
                ShotSetting.HEIGHT,
                6,
                ShotSetting.YAW,
                Math.toDegrees(Math.atan2(toward.y(), toward.x())),
                ShotSetting.PITCH,
                -5,
                ShotSetting.WHEEL_ONE_RPM,
                900,
                ShotSetting.WHEEL_TWO_RPM,
                900));

        Shot.Track knocked = shot.story().pieces().stream()
                .filter(track -> track.piece().at().equals(target.at()))
                .findFirst()
                .orElseThrow();
        Shot.Placed last = knocked.samples().get(knocked.samples().size() - 1);
        assertTrue("it was woken: " + knocked.samples(), knocked.samples().size() > 1);
        assertTrue(
                "it moved along the wall to " + last.at(),
                Math.abs(last.at().y() - target.at().y()) > 0.05);
        assertTrue(
                shot.story().impacts().stream().anyMatch(impact -> impact.hit().surface() == Surface.PIECE));
    }

    @Test
    public void withThePiecesOffNothingButTheFieldIsInPlay() {
        Shot.Finished shot = flown(with(ShotSetting.PIECES, 0));

        assertTrue(shot.story().pieces().isEmpty());
        assertTrue(
                shot.story().impacts().stream().noneMatch(impact -> impact.hit().surface() == Surface.PIECE));
    }

    @Test
    public void aTippedBlueHiveIsScoredInAtItsOtherEnd() {
        Shot.Finished shot = flown(with(ShotSetting.BLUE_HIVE_TIPPED, 1, ShotSetting.X, -55, ShotSetting.YAW, 0));

        assertEquals(
                shot.finish().toString(),
                new Shot.Resting.InCell(BLUE_AUDIENCE, "Blue", true),
                ((Shot.Finish.AtRest) shot.finish()).where());
    }

    @Test
    public void theFinestStepStaysWithinTheWorkAShotMayDo() {
        long began = System.nanoTime();
        Shot.Finished shot = flown(with(ShotSetting.STEP, 0.01));
        double spent = (System.nanoTime() - began) / 1e9;
        double simulated = shot.story().simulated().value();

        assertFalse(shot.finish().toString(), shot.finish() instanceof Shot.Finish.TooMuchWork);
        assertTrue(shot.story().work() <= Shot.MOST_WORK_PER_SECOND * simulated);
        System.out.println("the finest step: " + simulated + " s simulated in " + spent + " s, " + spent / simulated
                + " x real time, " + shot.story().work() + " integrations");
    }

    @Test
    public void everySettingAtEitherEndOfItsRangeStillLaunches() {
        for (ShotSetting setting : ShotSetting.values()) {
            for (double value : new double[] {setting.least(), setting.most()}) {
                String said = Shot.from(FIELD, with(setting, value)).fold(shot -> "", rule -> rule);
                assertEquals(setting + " at " + value, "", said);
            }
        }
    }
}
