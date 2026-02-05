package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {
    public Servo gateServo;
    public boolean gateIsOpen;

    private final double GATE_OPEN_POSITION = 0.55;
    private final double GATE_CLOSED_POSITION = 0.40;

    public void init(HardwareMap hw) {
        gateServo = hw.get(Servo.class, "gateServo");

        gateServo.setPosition(GATE_CLOSED_POSITION);
    }

    public void openGate() {
        gateServo.setPosition(GATE_OPEN_POSITION);
        gateIsOpen = true;
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED_POSITION);
        gateIsOpen = false;
    }
}
