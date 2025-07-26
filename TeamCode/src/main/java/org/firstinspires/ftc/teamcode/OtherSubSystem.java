package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SubSystem;

public class OtherSubSystem extends SubSystem {
    public OtherSubSystem(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        telemetry.addData("OtherSubSystem.init()", true);
    }

    @Override
    public void loop() {

    }

    @Override
    public boolean done() {
        return false;
    }
}
