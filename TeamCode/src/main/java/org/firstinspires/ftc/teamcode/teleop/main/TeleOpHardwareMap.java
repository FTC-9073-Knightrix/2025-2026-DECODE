package org.firstinspires.ftc.teamcode.teleop.main;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.vision.VisionSystem;

public abstract class TeleOpHardwareMap extends OpMode {
    // ---------------Hardware Map and Variables------------
    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();
    VisionSystem vision = new VisionSystem();
    public Servo hoodServo;
    public DcMotorEx outtakeMotor;

    public double hoodPosition = 0.0;


    static final double TICKS_PER_REV = 28;
    public double targetVelocityTicks = 0.0;

    @Override
    public void init() {
        drive.init(hardwareMap);
        vision.init(hardwareMap);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(hoodPosition);
    }
}
