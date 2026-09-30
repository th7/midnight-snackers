package org.firstinspires.ftc.teamcode.sim;

import java.util.Iterator;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.simcore.Drivebase;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;

public final class SimNoise {
    public static final Noise NONE = Noise.exact(
            Valid.value(Noise.Battery.of(SimDevices.BATTERY_VOLTS, 0, 0)),
            Valid.value(Noise.Loop.of(SimRunner.LOOP_SECONDS, 0, 0)));

    private SimNoise() {}

    public static String described(TeamRobot robot, Noise noise) {
        Drivebase drivebase = robot.drivebase();
        StringBuilder motors = new StringBuilder();
        Iterator<String> names = drivebase.motors().iterator();
        for (Noise.Motor motor : drivebase.perMotor(noise.motors())) {
            motors.append(String.format(
                    " %s kSVolts x%.3f kVVoltSecondsPerTick x%.3f kAVoltSecondsSquaredPerTick x%.3f",
                    names.next().toLowerCase(Locale.ROOT),
                    motor.kSVolts(),
                    motor.kVVoltSecondsPerTick(),
                    motor.kAVoltSecondsSquaredPerTick()));
        }
        Noise.Battery battery = noise.battery();
        return String.format(
                "%s, seed %d:%s; battery %.2f V sag %.2f V/power drain %.4f V/s; traction %.2f g;"
                        + " set down +-%.2f in +-%.1f deg; loop %.0f ms spread %.2f hiccups %.0f%%",
                robot.displayName(),
                noise.seed(),
                motors,
                battery.freshVolts(),
                battery.sagVoltsPerPower(),
                battery.drainVoltsPerSecond(),
                noise.traction().inPerS2() / Flight.GRAVITY_IN_PER_S2,
                noise.hand().inches(),
                Math.toDegrees(noise.hand().radians()),
                noise.loop().period().value() * 1000,
                noise.loop().spread(),
                noise.loop().hiccupChance() * 100);
    }
}
