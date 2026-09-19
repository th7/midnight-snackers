package org.firstinspires.ftc.teamcode.opmode;

import java.util.function.Function;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

public final class PlanOp extends AutoOp {
    private final String where;
    private final Function<Plans, PlanPart> plan;

    public PlanOp(Alliance alliance, String where, Function<Plans, PlanPart> plan) {
        super(alliance);
        this.where = where;
        this.plan = plan;
    }

    @Override
    public PlanPart getPlan() {
        return plan.apply(robot.plans);
    }

    @Override
    public String where() {
        return where;
    }
}
