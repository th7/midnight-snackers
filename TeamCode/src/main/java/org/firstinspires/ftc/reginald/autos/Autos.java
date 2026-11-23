package org.firstinspires.ftc.reginald.autos;

import org.firstinspires.ftc.framework.plans.Pause;
import org.firstinspires.ftc.framework.plans.Plan;
import org.firstinspires.ftc.framework.plans.PlanPart;
import org.firstinspires.ftc.framework.robot.Alliance;
import org.firstinspires.ftc.framework.robot.Auto;
import org.firstinspires.ftc.reginald.subsystems.Drive;
import org.firstinspires.ftc.reginald.subsystems.Intake;

public class Autos {
    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart driveForward(Drive drive) {
        return new Plan(drive.to(24, 0, 0));
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart forwardback(Drive drive, Pause pause) {
        return new Plan(drive.near(12, 0, 0), pause.forSeconds(10), drive.near(-12, 0, 0));
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart forwardLeftBackwardRight(Drive drive, Pause pause) {
        return new Plan(
                pause.forSeconds(5),
                drive.to(24, 0, 0),
                drive.to(24, 24, 0),
                drive.to(0, 24, 0),
                drive.to(0, 0, 0));
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart spinIntak(Intake intake) {
        return new Plan(intake.spin());
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart spinnyThing(Drive drive) {
        return new Plan(
                drive.near(0, -1, -Math.PI / 2),
                drive.near(-1, 0, -Math.PI),
                drive.near(0, 1, Math.PI / 2),
                drive.near(0, 0, 0),
                drive.near(12, 0, 0),
                drive.near(12, -6, 0),
                drive.near(0, 0, 0));
    }
}
