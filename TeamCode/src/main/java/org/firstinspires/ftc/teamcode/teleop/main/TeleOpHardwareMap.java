package org.firstinspires.ftc.teamcode.teleop.main;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpIntake;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpOuttake;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.vision.VisionSystem;

public abstract class TeleOpHardwareMap extends OpMode {
    // ---------------Hardware Map and Variables------------
    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();
    VisionSystem vision = new VisionSystem();
    TeleOpOuttake outtake = new TeleOpOuttake();
    TeleOpIntake intake = new TeleOpIntake();

    @Override
    public void init() {
        // Initializing hardware mechanism classes
        drive.init(hardwareMap);
        vision.init(hardwareMap);
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
    }
}
