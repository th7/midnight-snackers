package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class ShotFingerprintTest {
    private static final Ball POLLEN = Valid.value(Ball.of(2.8, 24.9, 0.634));
    private static final Ball NECTAR = Valid.value(Ball.of(3.6, 41.3, 0.634));
    private static final double R = POLLEN.radius();
    private static final String NOTHING = "0:cbf29ce484222325";
    private static final String NO_BOUNCE = "1:f8bb92ce26585846";
    private static final Colliders WALLS = Valid.value(Colliders.of(
            Valid.value(Field.of(
                    Valid.length(141.17),
                    Valid.length(11.64),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of())),
            List.of()));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Bounce bounce(double restitution, double friction, double grip) {
        return Valid.value(Bounce.of(restitution, friction, grip));
    }

    private static Surfaces plainFloor() {
        Bounce other = bounce(0.5, 0.3, 0.15);
        return Valid.value(Surfaces.of(bounce(0.55, 0.5, 0.2), other, other, other, other, other, 0, 4, 0, 0, 0));
    }

    private static Surfaces realistic() {
        Bounce other = bounce(0.5, 0.3, 0.15);
        return Valid.value(
                Surfaces.of(bounce(0.55, 0.5, 0.2), other, other, other, other, other, 0.038, 4, 0.1, 0.06, 0.04));
    }

    private static Air air() {
        return Valid.value(Air.of(1.204, Vec3.zero(), 0.45, 0.15, 2, 0.4, 0.012, 0.5));
    }

    private static World world(Air air, Surfaces surfaces, double longest, List<World.Piece> pieces) {
        return world(air, surfaces, 1e-4, longest, pieces);
    }

    private static World world(Air air, Surfaces surfaces, double step, double longest, List<World.Piece> pieces) {
        return Valid.value(World.of(WALLS, air, surfaces, seconds(step), seconds(longest), pieces));
    }

    private static List<World.Piece> aPollenRestingOnTwoNectar() {
        World.Piece left = new World.Piece(
                "Blue Nectar <1>", Field.Kind.NECTAR, Optional.of("Blue"), NECTAR, new Vec3(40, 0, 1.8), false);
        World.Piece right = new World.Piece(
                "Blue Nectar <2>", Field.Kind.NECTAR, Optional.of("Blue"), NECTAR, new Vec3(40, 3.6, 1.8), false);
        double above = StrictMath.sqrt((1.8 + R) * (1.8 + R) - 1.8 * 1.8);
        World.Piece top = new World.Piece(
                "Pollen <3>", Field.Kind.POLLEN, Optional.empty(), POLLEN, new Vec3(40, 1.8, 1.8 + above), false);
        return List.of(left, right, top);
    }

    private static Shot.Flying launch(World world, Vec3 at, Vec3 velocity, Vec3 spin) {
        return Valid.value(
                Shot.launch(world, POLLEN, Valid.value(WheelPair.Launched.of(at, velocity, spin, 0, 0, 0, 0))));
    }

    private static Shot inChunks(Shot.Flying flying, double chunk, double longest) {
        Shot shot = flying;
        double until = 0;
        while (shot instanceof Shot.Flying still && until < longest) {
            until += chunk;
            shot = still.advance(seconds(StrictMath.min(until, longest)));
        }
        return shot;
    }

    @Test
    public void aPieceKnockedOverAWallComesOutTheSameToTheBit() {
        World.Piece hanging = new World.Piece(
                "Red Nectar", Field.Kind.NECTAR, Optional.of("Red"), NECTAR, new Vec3(60, 0, 20), false);
        Shot shot = launch(
                        world(Air.vacuum(), plainFloor(), 1, List.of(hanging)),
                        new Vec3(40, 0, 20),
                        new Vec3(400, 0, 0),
                        Vec3.zero())
                .advance(seconds(1));

        assertEquals(
                new Fingerprint(
                        43021,
                        "5529:74d068edac1aa18f",
                        "359:3882a3b0f93d43d4",
                        NOTHING,
                        "21:0c4289558a51b59a",
                        "6:ad07f421afde4c08",
                        "165:5e349ea3a70c5b0d",
                        "36:fdb3cd134789e1b6"),
                Fingerprint.of(shot));
    }

    @Test
    public void aPieceRestingOnAnotherThatComesDownComesOutTheSameToTheBit() {
        Shot shot = launch(
                        world(Air.vacuum(), plainFloor(), 4, aPollenRestingOnTwoNectar()),
                        new Vec3(65, 3.6, R),
                        new Vec3(-120, 0, 0),
                        new Vec3(0, -120 / R, 0))
                .advance(seconds(4));

        assertEquals(
                new Fingerprint(
                        931613,
                        "22222:53ad876bc2039709",
                        "369:29f5ea56296c974e",
                        NOTHING,
                        "21:d5d2cb447e142c5a",
                        "6:fb0744a10650fe2d",
                        "23074:0c309e3a4614cfcd",
                        "36:87f0613df9146bd9"),
                Fingerprint.of(shot));
    }

    @Test
    public void piecesThatComeToRestAndFallAsleepOnRealisticSurfacesComeOutTheSameToTheBit() {
        Shot shot = launch(
                        world(air(), realistic(), 4, aPollenRestingOnTwoNectar()),
                        new Vec3(65, 3.6, R),
                        new Vec3(-120, 0, 0),
                        new Vec3(0, -120 / R, 0))
                .advance(seconds(4));

        assertEquals(
                new Fingerprint(
                        406551,
                        "14135:a6522fdfa0e2ffb2",
                        "428:3284b07145063b23",
                        NOTHING,
                        "21:9dc8ba04552de452",
                        "6:ac22c842ff2420f1",
                        "8690:9d73d600dbe803ea",
                        "56:695ce0e77b3abd8f"),
                Fingerprint.of(shot));
    }

    @Test
    public void aFastShotOnACoarseStepCutIntoPiecesComesOutTheSameToTheBit() {
        Shot shot = launch(
                        world(air(), realistic(), 2e-3, 3, aPollenRestingOnTwoNectar()),
                        new Vec3(-20, 1.8, 3),
                        new Vec3(400, 0, 0),
                        new Vec3(0, -60, 0))
                .advance(seconds(3));

        assertEquals(
                new Fingerprint(
                        42040,
                        "15744:25ab2237ccffab7f",
                        "433:63e4561d92c12266",
                        NOTHING,
                        "21:3652c510452d86b6",
                        "6:28dcaebf2ce77aec",
                        "12402:51751443040c8038",
                        "56:6f8cf1996b244fe0"),
                Fingerprint.of(shot));
    }

    @Test
    public void aPollenRattlingBetweenFixedPiecesUntilAPieceOfTimeRunsOutOfEventsComesOutTheSameToTheBit() {
        Bounce lively = bounce(0.95, 0.3, 0.15);
        Surfaces bouncy = Valid.value(Surfaces.of(lively, lively, lively, lively, lively, lively, 0, 1, 0, 0, 0));
        double apart = R + 1.8 + 1e-4;
        List<World.Piece> pieces = List.of(
                new World.Piece("Left", Field.Kind.NECTAR, Optional.empty(), NECTAR, new Vec3(-apart, 0, 5), true),
                new World.Piece("Right", Field.Kind.NECTAR, Optional.empty(), NECTAR, new Vec3(apart, 0, 5), true),
                new World.Piece(
                        "Under", Field.Kind.POLLEN, Optional.empty(), POLLEN, new Vec3(0, 0, 5 - 2 * R - 1e-4), true));
        Shot shot = launch(
                        world(Air.vacuum(), bouncy, 0.5, pieces),
                        new Vec3(0, 0, 5),
                        new Vec3(300, 0, -100),
                        new Vec3(0, 50, 0))
                .advance(seconds(0.5));

        assertEquals(
                new Fingerprint(
                        31240,
                        "6246:79c2cb9704cfd948",
                        "6435:df3cb673473a941b",
                        NOTHING,
                        "21:ccba5de31ab23dd6",
                        NO_BOUNCE,
                        "80:b698ee7df4c915ea",
                        "46:10464072ee49e9e8"),
                Fingerprint.of(shot));
    }

    @Test
    public void aBallRolledIntoTwoTouchingPiecesComesOutTheSameToTheBit() {
        World.Piece near =
                new World.Piece("Pollen <1>", Field.Kind.POLLEN, Optional.empty(), POLLEN, new Vec3(40, 0, R), false);
        World.Piece far = new World.Piece(
                "Pollen <2>", Field.Kind.POLLEN, Optional.empty(), POLLEN, new Vec3(40 + 2 * R, 0, R), false);
        Shot shot = launch(
                        world(Air.vacuum(), plainFloor(), 1.5, List.of(near, far)),
                        new Vec3(20, 0, R),
                        new Vec3(100, 0, 0),
                        new Vec3(0, 100 / R, 0))
                .advance(seconds(1.5));

        assertEquals(
                new Fingerprint(
                        242339,
                        "8402:faaefd1ee3cb735e",
                        "300:95eb08c32502dd7e",
                        NOTHING,
                        "21:ac119925b94b3d3d",
                        "6:f09165f74f4709ce",
                        "5424:a6327b74a8885429",
                        "36:a17bbea8a34b2c05"),
                Fingerprint.of(shot));
    }

    @Test
    public void aSpinningShotThroughTheAirAdvancedInOddChunksComesOutTheSameToTheBit() {
        Shot shot = inChunks(
                launch(
                        world(air(), plainFloor(), 3, List.of()),
                        new Vec3(50, 50, 5),
                        new Vec3(300, 280, -50),
                        new Vec3(100, -80, 200)),
                0.037,
                3);

        assertEquals(
                new Fingerprint(
                        173265,
                        "16853:d372e1d6a0b5fe40",
                        "597:7ea9a513bd97c395",
                        NOTHING,
                        "21:87ff77fcd61beb60",
                        "6:bff17f6b3025b6d2",
                        NOTHING,
                        "36:be46a320ed01a06a"),
                Fingerprint.of(shot));
    }

    @Test
    public void aSpinningShotOnRealisticSurfacesComesOutTheSameToTheBit() {
        Shot shot = launch(
                        world(air(), realistic(), 6, List.of()),
                        new Vec3(50, 50, 5),
                        new Vec3(300, 280, -50),
                        new Vec3(100, -80, 200))
                .advance(seconds(6));

        assertEquals(
                new Fingerprint(
                        250438,
                        "23602:73f70ab19919dcc9",
                        "417:8b5736024c84a3a8",
                        NOTHING,
                        "21:87ff77fcd61beb60",
                        "6:bff17f6b3025b6d2",
                        NOTHING,
                        "56:d8184749679b9eb9"),
                Fingerprint.of(shot));
    }
}
