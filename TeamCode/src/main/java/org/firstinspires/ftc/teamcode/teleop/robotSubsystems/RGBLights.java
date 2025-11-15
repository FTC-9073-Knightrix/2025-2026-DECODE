package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RGBLights {
    private Servo blinkin;

    public RGBLights(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(Servo.class, "blinkin");
    }

    public void setPattern(double value) {
        blinkin.setPosition(value);
    }
}