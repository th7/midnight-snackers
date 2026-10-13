package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

final class ShotSetup {
    static final String BLUE = "Blue";
    static final String RED = "Red";
    static final double MILLISECONDS_PER_SECOND = 1000;
    static final double PERCENT = 100;

    private ShotSetup() {}

    static Checked<Shot.Flying> shot(Field field, ShotSettings settings) {
        return pollen(settings)
                .then(pollen -> world(field, settings, pollen)
                        .then(world ->
                                wheels(settings, pollen).then(wheels -> Shot.launch(world, pollen, wheels.launch()))));
    }

    static final double SECONDS_PER_MINUTE = 60;

    static Checked<Shot.Preview> preview(Field field, ShotSettings settings) {
        return pollen(settings)
                .then(pollen -> wheels(settings, pollen)
                        .then(wheels -> air(settings)
                                .then(air -> Colliders.of(field, tipped(settings))
                                        .map(colliders -> preview(settings, pollen, wheels, air, colliders)))));
    }

    private static Shot.Preview preview(
            ShotSettings settings, Ball pollen, WheelPair wheels, Air air, Colliders colliders) {
        WheelPair.Launched launched = wheels.launch();
        List<Shot.Aim> aims = new ArrayList<>();
        for (Colliders.Mouth mouth : colliders.mouths()) {
            Vec3 toward = mouth.centre().minus(launched.at());
            aims.add(new Shot.Aim(
                    mouth.cell(),
                    mouth.alliance(),
                    mouth.upturned(),
                    StrictMath.toDegrees(StrictMath.atan2(toward.y(), toward.x())),
                    StrictMath.sqrt(toward.x() * toward.x() + toward.y() * toward.y()),
                    mouth.centre().z()));
        }
        double ticks = settings.value(ShotSetting.TICKS_PER_REV) / SECONDS_PER_MINUTE;
        return new Shot.Preview(
                launched,
                launched.kind(),
                launched.spinRpm(),
                air.spinParameter(pollen, launched.velocity(), launched.spin()),
                settings.value(ShotSetting.WHEEL_ONE_RPM) * ticks,
                settings.value(ShotSetting.WHEEL_TWO_RPM) * ticks,
                aims);
    }

    static Checked<Ball> pollen(ShotSettings settings) {
        return Ball.of(
                settings.value(ShotSetting.DIAMETER),
                settings.value(ShotSetting.MASS),
                settings.value(ShotSetting.INERTIA));
    }

    static Checked<WheelPair> wheels(ShotSettings settings, Ball pollen) {
        Checked<WheelPair.Inertia> inertia = settings.on(ShotSetting.WHEELS_SLOW)
                ? WheelPair.Inertia.Slows.of(WheelPair.Inertia.Slows.fromPoundSquareInches(
                                settings.value(ShotSetting.WHEEL_INERTIA)))
                        .map(ShotSetup::asInertia)
                : Checked.ok(new WheelPair.Inertia.KeepSpeed());
        return WheelPair.Flywheel.of(
                        settings.value(ShotSetting.WHEEL_ONE_DIAMETER), settings.value(ShotSetting.WHEEL_ONE_RPM))
                .then(one -> WheelPair.Flywheel.of(
                                settings.value(ShotSetting.WHEEL_TWO_DIAMETER),
                                settings.value(ShotSetting.WHEEL_TWO_RPM))
                        .then(two -> inertia.then(slowing -> WheelPair.of(
                                pollen,
                                new Vec3(
                                        settings.value(ShotSetting.X),
                                        settings.value(ShotSetting.Y),
                                        settings.value(ShotSetting.HEIGHT)),
                                settings.value(ShotSetting.YAW),
                                settings.value(ShotSetting.PITCH),
                                settings.value(ShotSetting.ROLL),
                                one,
                                two,
                                settings.value(ShotSetting.SPEED_EFFICIENCY),
                                settings.value(ShotSetting.SPIN_EFFICIENCY),
                                settings.value(ShotSetting.COMPRESSION),
                                slowing,
                                new Vec2(
                                        settings.value(ShotSetting.ROBOT_VX), settings.value(ShotSetting.ROBOT_VY))))));
    }

    private static WheelPair.Inertia asInertia(WheelPair.Inertia.Slows slows) {
        return slows;
    }

    static Checked<Air> air(ShotSettings settings) {
        return Air.densityOf(
                        settings.value(ShotSetting.TEMPERATURE),
                        settings.value(ShotSetting.ELEVATION),
                        settings.value(ShotSetting.HUMIDITY))
                .then(density -> Air.of(
                        density,
                        new Vec3(settings.value(ShotSetting.WIND_X), settings.value(ShotSetting.WIND_Y), 0),
                        settings.value(ShotSetting.DRAG),
                        settings.value(ShotSetting.DRAG_PER_SPIN),
                        settings.value(ShotSetting.LIFT_SLOPE),
                        settings.value(ShotSetting.LIFT_LIMIT),
                        settings.value(ShotSetting.SPIN_DOWN),
                        settings.value(ShotSetting.ADDED_MASS)));
    }

    static Checked<Bounce> bounce(
            ShotSettings settings, ShotSetting restitution, ShotSetting friction, ShotSetting grip) {
        return Bounce.of(settings.value(restitution), settings.value(friction), settings.value(grip));
    }

    static Checked<Surfaces> surfaces(ShotSettings settings) {
        return bounce(settings, ShotSetting.FLOOR_BOUNCE, ShotSetting.FLOOR_FRICTION, ShotSetting.FLOOR_GRIP)
                .then(floor -> bounce(
                                settings, ShotSetting.WALL_BOUNCE, ShotSetting.WALL_FRICTION, ShotSetting.WALL_GRIP)
                        .then(wall -> bounce(
                                        settings,
                                        ShotSetting.FRAME_BOUNCE,
                                        ShotSetting.FRAME_FRICTION,
                                        ShotSetting.FRAME_GRIP)
                                .then(frame -> bounce(
                                                settings,
                                                ShotSetting.FLOWER_BOUNCE,
                                                ShotSetting.FLOWER_FRICTION,
                                                ShotSetting.FLOWER_GRIP)
                                        .then(flower -> bounce(
                                                        settings,
                                                        ShotSetting.CELL_BOUNCE,
                                                        ShotSetting.CELL_FRICTION,
                                                        ShotSetting.CELL_GRIP)
                                                .then(cell -> bounce(
                                                                settings,
                                                                ShotSetting.PIECE_BOUNCE,
                                                                ShotSetting.PIECE_FRICTION,
                                                                ShotSetting.PIECE_GRIP)
                                                        .then(piece -> Surfaces.of(
                                                                floor,
                                                                wall,
                                                                frame,
                                                                flower,
                                                                cell,
                                                                piece,
                                                                settings.value(ShotSetting.BOUNCE_LOSS),
                                                                settings.value(ShotSetting.BOUNCE_THRESHOLD),
                                                                settings.value(ShotSetting.IMPACT_TWIST),
                                                                settings.value(ShotSetting.SPIN_TWIST),
                                                                settings.value(ShotSetting.ROLLING_RESISTANCE))))))));
    }

    static List<String> tipped(ShotSettings settings) {
        List<String> tipped = new ArrayList<>();
        if (settings.on(ShotSetting.BLUE_HIVE_TIPPED)) {
            tipped.add(BLUE);
        }
        if (settings.on(ShotSetting.RED_HIVE_TIPPED)) {
            tipped.add(RED);
        }
        return tipped;
    }

    static final int SETTING_DOWN_PASSES = 200;
    static final double SET_DOWN_IN = 1e-9;

    private static final class Spot {
        private final World.Piece piece;
        private Vec3 at;

        private Spot(World.Piece piece) {
            this.piece = piece;
            this.at = piece.at();
        }
    }

    static List<World.Piece> setDownClear(Colliders colliders, List<World.Piece> pieces) {
        List<Spot> spots = new ArrayList<>();
        for (World.Piece piece : pieces) {
            spots.add(new Spot(piece));
        }
        boolean moved = true;
        for (int pass = 0; pass < SETTING_DOWN_PASSES && moved; pass++) {
            moved = false;
            for (Spot spot : spots) {
                if (spot.piece.fixed()) {
                    continue;
                }
                for (Colliders.Near near :
                        colliders.near(spot.at, spot.piece.shape().radiusLength(), 0)) {
                    if (near.gap() < -SET_DOWN_IN) {
                        spot.at = spot.at.along(near.normal(), -near.gap());
                        moved = true;
                    }
                }
                for (Spot other : spots) {
                    Vec3 apart = spot.at.minus(other.at);
                    double distance = StrictMath.sqrt(apart.dot(apart));
                    double over =
                            spot.piece.shape().radius() + other.piece.shape().radius() - distance;
                    if (other != spot && distance > 0 && over > SET_DOWN_IN) {
                        Vec3 away = apart.times(1 / distance);
                        if (other.piece.fixed()) {
                            spot.at = spot.at.along(away, over);
                        } else {
                            spot.at = spot.at.along(away, over / 2);
                            other.at = other.at.along(away, -over / 2);
                        }
                        moved = true;
                    }
                }
            }
        }
        List<World.Piece> placed = new ArrayList<>();
        for (Spot spot : spots) {
            World.Piece piece = spot.piece;
            placed.add(new World.Piece(
                    piece.name(), piece.kind(), piece.alliance(), piece.shape(), spot.at, piece.fixed()));
        }
        return placed;
    }

    static Checked<List<World.Piece>> pieces(Field field, ShotSettings settings, Ball pollen) {
        List<World.Piece> pieces = new ArrayList<>();
        if (!settings.on(ShotSetting.PIECES)) {
            return Checked.ok(pieces);
        }
        List<String> tipped = tipped(settings);
        int numbered = 0;
        for (Field.Piece piece : field.movedPieces()) {
            numbered++;
            if (piece.place() instanceof Field.Place.InCell held
                    && tipped.contains(held.cell().alliance())) {
                continue;
            }
            Checked<Ball> shape = piece.kind() == Field.Kind.POLLEN
                    ? Checked.ok(pollen)
                    : Ball.of(2 * piece.radius(), settings.value(ShotSetting.NECTAR_MASS), pollen.inertiaFactor());
            if (shape instanceof Checked.Rejected<Ball> rejected) {
                return Checked.rejected(rejected.rule());
            }
            String name = piece.name() + " <" + numbered + ">";
            boolean fixed = piece.place() instanceof Field.Place.InFlower;
            Optional<String> alliance = piece.alliance();
            shape.fold(
                    s -> pieces.add(new World.Piece(name, piece.kind(), alliance, s, piece.at(), fixed)),
                    rule -> false);
        }
        return Checked.ok(pieces);
    }

    static Checked<World> world(Field field, ShotSettings settings, Ball pollen) {
        return Colliders.of(field, tipped(settings))
                .then(colliders -> air(settings)
                        .then(air -> surfaces(settings)
                                .then(surfaces -> pieces(field, settings, pollen)
                                        .map(laid -> setDownClear(colliders, laid))
                                        .then(pieces -> Seconds.of(
                                                        settings.value(ShotSetting.STEP) / MILLISECONDS_PER_SECOND)
                                                .then(step -> Seconds.of(settings.value(ShotSetting.LONGEST))
                                                        .then(longest -> World.of(
                                                                colliders, air, surfaces, step, longest, pieces)))))));
    }
}
