package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {
    public Servo gateServo;
    public boolean gateIsOpen;

    private final double GATE_OPEN_POSITION = 0.6; // TODO: adjust these values based on testing
    private final double GATE_CLOSED_POSITION = 0.45;

    public void init(HardwareMap hw) {
        gateServo = hw.get(Servo.class, "gateServo");

        gateServo.setPosition(GATE_CLOSED_POSITION);
        gateIsOpen = false;
    }

    public void runGate(boolean shootTrigger) {
        if (shootTrigger && !gateIsOpen) {
            gateServo.setPosition(GATE_OPEN_POSITION);
            gateIsOpen = true;
        } else if (!shootTrigger && gateIsOpen) {
            gateServo.setPosition(GATE_CLOSED_POSITION);
            gateIsOpen = false;
        }
    }
}
