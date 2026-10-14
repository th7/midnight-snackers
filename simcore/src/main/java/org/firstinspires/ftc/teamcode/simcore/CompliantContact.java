package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

final class CompliantContact {
    static final String STICK_SOURCE = "RK4 shrinks a decay y' = -k y while k times its step is under about 2.785,"
            + " where its region of absolute stability meets the negative real axis, so regularised friction"
            + " in stick is stable while the step is under 2.785 v_reg / (sum of mu N (1/m + r^2/I))";
    static final double STICK_STABLE_WITHIN = 2.785;
    static final double STICK_SAFETY = 2;

    record Push(Vec3 acceleration, Vec3 spinRate) {}

    private record Pressed(Surroundings.Squeeze squeeze, double newtons) {}

    private CompliantContact() {}

    static Push push(Ball ball, Vec3 velocity, Vec3 spin, List<Surroundings.Squeeze> squeezes, Bodies.Step step) {
        double kilograms = ball.kilograms();
        double inch = Length.METRES_PER_INCH;
        List<Pressed> pressed = new ArrayList<>();
        double gripping = 0;
        for (Surroundings.Squeeze squeeze : squeezes) {
            Compliance compliance = squeeze.compliance();
            double closing = -velocity.minus(squeeze.velocity()).dot(squeeze.normal()) * inch;
            double newtons =
                    compliance.newtonsPerMetre() * squeeze.depth() * inch + compliance.dampingFor(ball) * closing;
            if (newtons > 0) {
                pressed.add(new Pressed(squeeze, newtons));
                gripping += compliance.friction() * newtons;
            }
        }
        double perKilogram = (1 + 1 / ball.inertiaFactor()) / kilograms;
        double stable = STICK_SAFETY * step.seconds().value() * gripping * perKilogram / STICK_STABLE_WITHIN;
        Vec3 force = Vec3.zero();
        Vec3 torque = Vec3.zero();
        for (Pressed each : pressed) {
            Surroundings.Squeeze squeeze = each.squeeze();
            Vec3 normal = squeeze.normal();
            Vec3 arm = squeeze.arm();
            Vec3 slip = velocity.plus(spin.cross(arm)).minus(squeeze.velocity());
            Vec3 across = slip.minus(normal.times(slip.dot(normal))).times(inch);
            double regularised = StrictMath.max(squeeze.compliance().regularisation(), stable);
            double slipping = StrictMath.sqrt(across.dot(across));
            double grip = squeeze.compliance().friction() * each.newtons() / StrictMath.max(slipping, regularised);
            Vec3 friction = across.times(-grip);
            force = force.plus(normal.times(each.newtons())).plus(friction);
            torque = torque.plus(arm.times(inch).cross(friction));
        }
        double radius = ball.radiusMetres();
        return new Push(
                force.times(1 / (kilograms * inch)),
                torque.times(1 / (ball.inertiaFactor() * kilograms * radius * radius)));
    }
}
