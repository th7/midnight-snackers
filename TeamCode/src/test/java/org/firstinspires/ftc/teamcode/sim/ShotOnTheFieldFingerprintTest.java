package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import java.util.EnumMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Fingerprint;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting;
import org.firstinspires.ftc.teamcode.simcore.ShotSettings;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class ShotOnTheFieldFingerprintTest {
    private static final Field FIELD = SimPlacement.FIELD;
    private static final String NOTHING = "0:cbf29ce484222325";
    private static final String NO_BOUNCE = "1:f8bb92ce26585846";
    private static final Fingerprint THE_DEFAULT_SHOT = new Fingerprint(
            170583,
            "9994:dd962dd7b71e4e64",
            "1011:22ef2156216a6802",
            "40:7566ae2788a7f902",
            "21:653a638da19b4fdb",
            NO_BOUNCE,
            "3104:b7275680336a1a20",
            "152:e16c706ecece6e37");

    private static ShotSettings with(Object... pairs) {
        Map<ShotSetting, Double> set = new EnumMap<>(ShotSetting.class);
        for (int i = 0; i < pairs.length; i += 2) {
            set.put((ShotSetting) pairs[i], ((Number) pairs[i + 1]).doubleValue());
        }
        return Valid.value(ShotSettings.of(set));
    }

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Shot flown(ShotSettings settings) {
        return Valid.value(Shot.from(FIELD, settings)).advance(seconds(25));
    }

    private static Shot inTheBenchsChunks(ShotSettings settings) {
        Shot shot = Valid.value(Shot.from(FIELD, settings));
        double until = 0;
        while (shot instanceof Shot.Flying still) {
            until += SimLaunches.CHUNK_SECONDS;
            shot = still.advance(seconds(until));
        }
        return shot;
    }

    @Test
    public void theDefaultShotComesOutTheSameToTheBitAdvancedAsTheBenchAdvancesIt() {
        assertEquals(THE_DEFAULT_SHOT, Fingerprint.of(inTheBenchsChunks(ShotSettings.defaults())));
    }

    @Test
    public void theDefaultShotComesOutTheSameToTheBitAdvancedAllAtOnce() {
        assertEquals(THE_DEFAULT_SHOT, Fingerprint.of(flown(ShotSettings.defaults())));
    }

    @Test
    public void aPollenLaunchedAtALoosePieceComesOutTheSameToTheBit() {
        Field.Piece target = FIELD.loosePieces().get(0);
        Vec3 toward = new Vec3(StrictMath.signum(target.at().x()), 0, 0);
        Vec3 from = target.at().along(toward, -30).plus(new Vec3(0, -2, 0));
        Shot shot = flown(with(
                ShotSetting.X,
                from.x(),
                ShotSetting.Y,
                from.y(),
                ShotSetting.HEIGHT,
                6,
                ShotSetting.YAW,
                StrictMath.toDegrees(StrictMath.atan2(toward.y(), toward.x())),
                ShotSetting.PITCH,
                -5,
                ShotSetting.WHEEL_ONE_RPM,
                900,
                ShotSetting.WHEEL_TWO_RPM,
                900));

        assertEquals(
                new Fingerprint(
                        231506,
                        "15312:8993238c390bc160",
                        "602:be49305ae15f701a",
                        NOTHING,
                        "21:ca16d61ae8b79a15",
                        "6:77ae9c079eb9e284",
                        "3296:3b73bff38fc7b65d",
                        "56:e829b6e182b0948f"),
                Fingerprint.of(shot));
    }

    @Test
    public void theDefaultShotOnACoarseStepCutIntoPiecesComesOutTheSameToTheBit() {
        assertEquals(
                new Fingerprint(
                        20878,
                        "9912:8a1cb79d42668206",
                        "924:446348b761878756",
                        "40:58040981811494a5",
                        "21:f7a3f20abe3ca7f3",
                        NO_BOUNCE,
                        "3096:864deebbc2d78c94",
                        "152:d11996c5aa07bca1"),
                Fingerprint.of(flown(with(ShotSetting.STEP, 2))));
    }

    @Test
    public void aPollenThatClearsAWallComesOutTheSameToTheBit() {
        Shot shot = flown(with(
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

        assertEquals(
                new Fingerprint(
                        2306,
                        "336:45cc6ad9e5373b7f",
                        NOTHING,
                        NOTHING,
                        "21:b3487176ed223c5f",
                        NO_BOUNCE,
                        "1008:17a86cef463938ef",
                        "44:052fc6b266d2c4d9"),
                Fingerprint.of(shot));
    }
}
