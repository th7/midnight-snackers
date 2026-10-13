package org.firstinspires.ftc.teamcode.simcore;

import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.AIR;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.CONTACT;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.FIELD;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.FLIGHT;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.LAUNCHER;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.POLLEN;
import static org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group.SIMULATION;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting.Group;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting.OnSurface;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting.Property;
import org.junit.Test;

public class ShotSettingTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static String amount(
            ShotSetting setting, Group group, double byDefault, double least, double most, String unit) {
        return row(setting.name(), group, "amount", byDefault, least, most, unit);
    }

    private static String toggle(ShotSetting setting, Group group, double byDefault) {
        return row(setting.name(), group, "switch", byDefault, 0, 1, "");
    }

    private static String row(
            String name, Group group, String kind, double byDefault, double least, double most, String unit) {
        return name + " in " + group + ", " + kind + ", " + byDefault + " [" + least + ", " + most + "] '" + unit + "'";
    }

    private static String kindOf(ShotSetting setting) {
        ShotSetting.Kind kind = setting.kind();
        if (kind instanceof OnSurface on) {
            return on.surface() + " " + on.property();
        }
        return kind instanceof ShotSetting.Switch ? "switch" : "amount";
    }

    private static String rowOf(ShotSetting setting) {
        return row(
                setting.name(),
                setting.group(),
                kindOf(setting),
                setting.byDefault(),
                setting.least(),
                setting.most(),
                setting.unit());
    }

    private static List<String> surfaceRows(Surface surface, double bounce, double friction, double grip) {
        String name = surface.name();
        return List.of(
                row(name + "_BOUNCE", Group.BOUNCE, surface + " BOUNCE", bounce, 0, 1, ""),
                row(name + "_FRICTION", Group.BOUNCE, surface + " FRICTION", friction, 0, 1.5, ""),
                row(name + "_GRIP", Group.BOUNCE, surface + " GRIP", grip, 0, 1, ""));
    }

    private static List<ShotSetting> switches() {
        List<ShotSetting> switches = new ArrayList<>();
        for (ShotSetting setting : ShotSetting.values()) {
            if (setting.kind() instanceof ShotSetting.Switch) {
                switches.add(setting);
            }
        }
        return switches;
    }

    @Test
    public void theSettingsAreTheDesignsWithItsDefaultsRangesAndUnitsInItsOrder() {
        List<String> expected = new ArrayList<>(List.of(
                amount(ShotSetting.X, LAUNCHER, 55, -69, 69, "in"),
                amount(ShotSetting.Y, LAUNCHER, -12.76, -69, 69, "in"),
                amount(ShotSetting.HEIGHT, LAUNCHER, 14, 2, 40, "in"),
                amount(ShotSetting.YAW, LAUNCHER, 180, -180, 180, "deg"),
                amount(ShotSetting.PITCH, LAUNCHER, 70, -30, 90, "deg"),
                amount(ShotSetting.ROLL, LAUNCHER, 0, -180, 180, "deg"),
                amount(ShotSetting.WHEEL_ONE_RPM, LAUNCHER, 900, 0, 6000, "rpm"),
                amount(ShotSetting.WHEEL_TWO_RPM, LAUNCHER, 1200, 0, 6000, "rpm"),
                amount(ShotSetting.WHEEL_ONE_DIAMETER, LAUNCHER, 3.78, 1.5, 6, "in"),
                amount(ShotSetting.WHEEL_TWO_DIAMETER, LAUNCHER, 3.78, 1.5, 6, "in"),
                amount(ShotSetting.TICKS_PER_REV, LAUNCHER, 28, 1, 10000, "ticks"),
                amount(ShotSetting.SPEED_EFFICIENCY, LAUNCHER, 1, 0.3, 1, ""),
                amount(ShotSetting.SPIN_EFFICIENCY, LAUNCHER, 1, 0.3, 1, ""),
                amount(ShotSetting.COMPRESSION, LAUNCHER, 0, 0, 0.6, "in"),
                toggle(ShotSetting.WHEELS_SLOW, LAUNCHER, 0),
                amount(ShotSetting.WHEEL_INERTIA, LAUNCHER, 0.95, 0.05, 5, "lb in^2"),
                amount(ShotSetting.ROBOT_VX, LAUNCHER, 0, -120, 120, "in/s"),
                amount(ShotSetting.ROBOT_VY, LAUNCHER, 0, -120, 120, "in/s"),
                amount(ShotSetting.DIAMETER, POLLEN, 2.8, 2.6, 3.0, "in"),
                amount(ShotSetting.MASS, POLLEN, 24.9, 20, 30, "g"),
                amount(ShotSetting.INERTIA, POLLEN, 0.634, 0.4, Ball.THIN_SHELL, "of m r^2"),
                toggle(ShotSetting.PIECES, POLLEN, 1),
                amount(ShotSetting.NECTAR_MASS, POLLEN, 41.3, 30, 55, "g"),
                toggle(ShotSetting.BLUE_HIVE_TIPPED, FIELD, 0),
                toggle(ShotSetting.RED_HIVE_TIPPED, FIELD, 0),
                amount(ShotSetting.TEMPERATURE, AIR, 20, -10, 45, "C"),
                amount(ShotSetting.ELEVATION, AIR, 0, -100, 3000, "m"),
                amount(ShotSetting.HUMIDITY, AIR, 0, 0, 100, "%"),
                amount(ShotSetting.WIND_X, AIR, 0, -200, 200, "in/s"),
                amount(ShotSetting.WIND_Y, AIR, 0, -200, 200, "in/s"),
                amount(ShotSetting.DRAG, FLIGHT, 0.45, 0, 1, ""),
                amount(ShotSetting.DRAG_PER_SPIN, FLIGHT, 0.15, 0, 0.5, ""),
                amount(ShotSetting.LIFT_SLOPE, FLIGHT, 2, 0.5, 5, ""),
                amount(ShotSetting.LIFT_LIMIT, FLIGHT, 0.4, 0.1, 1, ""),
                amount(ShotSetting.SPIN_DOWN, FLIGHT, 0.012, 0, 0.05, ""),
                amount(ShotSetting.ADDED_MASS, FLIGHT, 0.5, 0, 1, "")));
        expected.addAll(surfaceRows(Surface.FLOOR, 0.55, 0.50, 0.20));
        expected.addAll(surfaceRows(Surface.WALL, 0.50, 0.30, 0.15));
        expected.addAll(surfaceRows(Surface.FRAME, 0.62, 0.35, 0.15));
        expected.addAll(surfaceRows(Surface.FLOWER, 0.55, 0.35, 0.15));
        expected.addAll(surfaceRows(Surface.CELL, 0.50, 0.30, 0.15));
        expected.addAll(surfaceRows(Surface.PIECE, 0.55, 0.30, 0.15));
        expected.addAll(List.of(
                amount(ShotSetting.BOUNCE_LOSS, CONTACT, 0.038, 0, 0.2, "per 100 in/s"),
                amount(ShotSetting.BOUNCE_THRESHOLD, CONTACT, 4, 0.5, 25, "in/s"),
                amount(ShotSetting.IMPACT_TWIST, CONTACT, 0.1, 0, 0.3, "in"),
                amount(ShotSetting.SPIN_TWIST, CONTACT, 0.06, 0, 0.2, "in"),
                amount(ShotSetting.ROLLING_RESISTANCE, CONTACT, 0.04, 0, 0.2, ""),
                amount(ShotSetting.STEP, SIMULATION, 0.1, 0.01, 2, "ms"),
                amount(ShotSetting.LONGEST, SIMULATION, 8, 0.5, 20, "s")));

        List<String> actual = new ArrayList<>();
        for (ShotSetting setting : ShotSetting.values()) {
            actual.add(rowOf(setting));
        }

        assertEquals(String.join("\n", expected), String.join("\n", actual));
    }

    @Test
    public void thePollensInertiaGoesAsHighAsAThinShellsAndNoHigher() {
        assertEquals(Ball.THIN_SHELL, ShotSetting.INERTIA.most(), 0);
        assertTrue(Ball.of(2.8, 24.9, ShotSetting.INERTIA.most()) instanceof Checked.Ok);
        assertRejected(String.valueOf(Ball.THIN_SHELL), ShotSetting.INERTIA.admits(Math.nextUp(Ball.THIN_SHELL)));
    }

    @Test
    public void everySettingSaysWhatItIsAndHasRoomToMoveAroundItsDefault() {
        for (ShotSetting setting : ShotSetting.values()) {
            assertFalse(setting.name(), setting.label().isBlank());
            assertFalse(setting.name(), setting.says().isBlank());
            assertTrue(setting.name() + " says a sentence", setting.says().endsWith("."));
            assertEquals(setting.name(), setting.unit().strip(), setting.unit());
            assertTrue(setting.name() + " is at least its least", setting.byDefault() >= setting.least());
            assertTrue(setting.name() + " is at most its most", setting.byDefault() <= setting.most());
            assertTrue(setting.name() + " has room to move", setting.least() < setting.most());
        }
    }

    @Test
    public void noTwoSettingsShareALabelSoARefusalNamesOne() {
        Set<String> labels = new HashSet<>();
        for (ShotSetting setting : ShotSetting.values()) {
            assertTrue(setting.label(), labels.add(setting.label().toLowerCase(Locale.ROOT)));
        }
    }

    @Test
    public void everyGroupSaysWhatItHoldsAndOnlyTheFirstThreeStartOpen() {
        for (Group group : Group.values()) {
            assertFalse(group.name(), group.label().isBlank());
            assertFalse(group.name(), group.says().isBlank());
            assertEquals(group.name().toLowerCase(Locale.ROOT), group.asked());
        }
        assertEquals("Launcher", LAUNCHER.label());
        assertEquals("Pollen and pieces", POLLEN.label());
        assertEquals("Field", FIELD.label());
        assertEquals("Air", AIR.label());
        assertEquals("Flight model", FLIGHT.label());
        assertEquals("Bounce", Group.BOUNCE.label());
        assertEquals("Contact", CONTACT.label());
        assertEquals("Simulation", SIMULATION.label());

        Set<Group> open = EnumSet.noneOf(Group.class);
        for (Group group : Group.values()) {
            if (group.openByDefault()) {
                open.add(group);
            }
        }
        assertEquals(EnumSet.of(LAUNCHER, POLLEN, FIELD), open);
    }

    @Test
    public void everyGroupHoldsASettingAndTheSettingsComeGroupByGroup() {
        Set<Group> holding = EnumSet.noneOf(Group.class);
        Group last = LAUNCHER;
        for (ShotSetting setting : ShotSetting.values()) {
            holding.add(setting.group());
            assertTrue(
                    setting + " in " + setting.group() + " comes after " + last,
                    setting.group().compareTo(last) >= 0);
            last = setting.group();
        }
        assertEquals(EnumSet.allOf(Group.class), holding);
    }

    @Test
    public void aSettingIsNamedByWhatItIsAskedForAsAndByNothingElse() {
        for (ShotSetting setting : ShotSetting.values()) {
            assertEquals(setting.name().toLowerCase(Locale.ROOT), setting.asked());
            assertSame(setting, Valid.value(ShotSetting.named(setting.asked())));
        }
        assertEquals("wheel_one_rpm", ShotSetting.WHEEL_ONE_RPM.asked());

        Checked<ShotSetting> unknown = ShotSetting.named("gravity");
        assertRejected("a launch setting is x, y, height, yaw, pitch, roll, ", unknown);
        assertRejected("not 'gravity'", unknown);
        for (ShotSetting setting : ShotSetting.values()) {
            assertRejected(setting.asked(), unknown);
        }
        assertRejected("'PITCH'", ShotSetting.named("PITCH"));
        assertRejected("''", ShotSetting.named(""));
    }

    @Test
    public void aValueOutsideItsRangeIsRefusedNamingTheSettingItsRangeItsUnitAndTheValue() {
        for (ShotSetting setting : ShotSetting.values()) {
            for (double wrong : List.of(
                    Math.nextDown(setting.least()),
                    Math.nextUp(setting.most()),
                    setting.least() - 1,
                    setting.most() + 1,
                    Double.NaN,
                    Double.POSITIVE_INFINITY,
                    Double.NEGATIVE_INFINITY)) {
                Checked<Double> refused = setting.admits(wrong);
                assertRejected(setting.label(), refused);
                assertRejected(String.valueOf(setting.least()), refused);
                assertRejected(String.valueOf(setting.most()), refused);
                assertRejected(setting.unit(), refused);
                assertRejected(String.valueOf(wrong), refused);
            }
        }
        assertRejected("Pitch is from -30.0 to 90.0 deg, not 95.0", ShotSetting.PITCH.admits(95));
        assertRejected("Drag is from 0.0 to 1.0, not 1.5", ShotSetting.DRAG.admits(1.5));
    }

    @Test
    public void aValueAtEitherEndOfItsRangeOrAtItsDefaultIsTaken() {
        for (ShotSetting setting : ShotSetting.values()) {
            for (double right : List.of(setting.least(), setting.byDefault(), setting.most())) {
                assertEquals(setting.name(), right, Valid.value(setting.admits(right)), 0);
            }
        }
        assertEquals(-12.5, Valid.value(ShotSetting.Y.admits(-12.5)), 0);
    }

    @Test
    public void aNegativeZeroIsTakenAsZero() {
        double taken = Valid.value(ShotSetting.ROLL.admits(-0.0));

        assertEquals(0, Double.compare(0.0, taken));
    }

    @Test
    public void theSwitchesAreOffOrOnAndNothingBetween() {
        assertEquals(
                List.of(
                        ShotSetting.WHEELS_SLOW,
                        ShotSetting.PIECES,
                        ShotSetting.BLUE_HIVE_TIPPED,
                        ShotSetting.RED_HIVE_TIPPED),
                switches());
        for (ShotSetting setting : switches()) {
            assertEquals(setting.name(), 0, setting.least(), 0);
            assertEquals(setting.name(), 1, setting.most(), 0);
            assertTrue(setting.name(), setting.byDefault() == 0 || setting.byDefault() == 1);
            assertEquals(setting.name(), "", setting.unit());
            assertEquals(0, Valid.value(setting.admits(0)), 0);
            assertEquals(1, Valid.value(setting.admits(1)), 0);
            for (double between : List.of(0.5, Math.nextUp(0.0), Math.nextDown(1.0))) {
                Checked<Double> refused = setting.admits(between);
                assertRejected(setting.label(), refused);
                assertRejected("0.0", refused);
                assertRejected("1.0", refused);
                assertRejected(String.valueOf(between), refused);
            }
        }
        assertRejected("Wheels slow down is 0.0 for off or 1.0 for on, not 0.5", ShotSetting.WHEELS_SLOW.admits(0.5));
    }

    @Test
    public void theBounceGroupIsAGridOfEverySurfaceByBounceFrictionAndGrip() {
        assertEquals(3, Property.values().length);
        assertEquals(6, Surface.values().length);
        Set<OnSurface> cells = new HashSet<>();
        for (ShotSetting setting : ShotSetting.values()) {
            boolean inTheGrid = setting.kind() instanceof OnSurface;
            assertEquals(setting.name(), setting.group() == Group.BOUNCE, inTheGrid);
            if (setting.kind() instanceof OnSurface on) {
                assertTrue(setting + " is the only " + on, cells.add(on));
                assertEquals(on.surface().name() + "_" + on.property().name(), setting.name());
            }
        }
        assertEquals(18, cells.size());
        for (Surface surface : Surface.values()) {
            for (Property property : Property.values()) {
                assertTrue(surface + " " + property, cells.contains(new OnSurface(surface, property)));
            }
        }
    }

    @Test
    public void everyPropertySaysWhatItIsAndIsAskedForByItsLowerCaseName() {
        assertEquals(
                List.of("bounce", "friction", "grip"),
                List.of(Property.BOUNCE.asked(), Property.FRICTION.asked(), Property.GRIP.asked()));
        for (Property property : Property.values()) {
            assertFalse(property.name(), property.label().isBlank());
            assertFalse(property.name(), property.says().isBlank());
        }
    }

    @Test
    public void aSurfacesBounceSaysTheSpeedItIsMeasuredAt() {
        for (ShotSetting setting : ShotSetting.values()) {
            if (setting.kind() instanceof OnSurface on && on.property() == Property.BOUNCE) {
                assertTrue(setting + ": " + setting.says(), setting.says().contains("244 in/s"));
            }
        }
        assertTrue(ShotSetting.BOUNCE_LOSS.says().contains("244 in/s"));
    }

    @Test
    public void theSettingsSayWhereTheirDefaultsComeFrom() {
        assertTrue(ShotSetting.DRAG.says(), ShotSetting.DRAG.says().contains("26-hole pickleball"));
        assertTrue(ShotSetting.DRAG.says(), ShotSetting.DRAG.says().contains("0.45"));
        assertTrue(
                ShotSetting.TICKS_PER_REV.says(),
                ShotSetting.TICKS_PER_REV.says().contains("only changes what the form shows"));
        assertTrue(
                ShotSetting.WHEEL_INERTIA.says(),
                ShotSetting.WHEEL_INERTIA.says().contains("Wheels slow down"));
    }
}
