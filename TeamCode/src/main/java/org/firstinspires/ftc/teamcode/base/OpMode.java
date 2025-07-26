package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.*;
import org.reflections.Reflections;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.reflections.scanners.Scanners;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    Arm arm;
    Drive drive;
    OtherSubSystem otherSubSystem;

    @Override
    public void init() {
        arm = new Arm(hardwareMap, runtime, telemetry);
        arm.init();
        drive = new Drive(hardwareMap, runtime, telemetry);
        drive.init();
        otherSubSystem = new OtherSubSystem(hardwareMap, runtime, telemetry);
        otherSubSystem.init();
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public void loop() {
        arm.loop();
        drive.loop();
        otherSubSystem.loop();
    }
}
