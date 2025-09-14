package org.firstinspires.ftc.teamcode.teleop.main;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    @Override
    public void init() {super.init();}
    // put robot methods you will run in the teleOp here
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
    }
}
