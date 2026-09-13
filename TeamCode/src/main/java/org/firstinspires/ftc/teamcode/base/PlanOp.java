package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

import java.util.function.Function;

/**
 * An auto whose plan is chosen when it is made: one of these per {@link Auto} annotation, so a
 * plan needs no op mode class of its own.
 */
public final class PlanOp extends AutoOp {
    private final String where;
    private final Function<Plans, PlanPart> plan;

    /**
     * @param where where a person finds the plan, e.g. {@code Plans.driveForward()}
     * @param plan  the plan to run, from the plans this op mode builds in {@link #init()}
     */
    public PlanOp(Alliance alliance, String where, Function<Plans, PlanPart> plan) {
        super(alliance);
        this.where = where;
        this.plan = plan;
    }

    @Override
    public PlanPart getPlan() {
        return plan.apply(plans);
    }

    @Override
    public String where() {
        return where;
    }
}
