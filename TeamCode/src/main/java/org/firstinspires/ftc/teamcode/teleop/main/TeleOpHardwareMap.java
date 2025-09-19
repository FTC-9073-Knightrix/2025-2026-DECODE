package org.firstinspires.ftc.teamcode.teleop.main;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;

public abstract class TeleOpHardwareMap extends OpMode {
    // ---------------Hardware Map and Variables------------
    boolean rb = gamepad1.right_bumper;
    boolean lb = gamepad1.left_bumper;
    double leftY = -gamepad1.left_stick_y;
    double leftX = gamepad1.left_stick_x;
    double rightX = gamepad1.right_stick_x * .8;
    boolean yButton = gamepad1.y;


    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();
    @Override
    public void init() {
        drive.init(hardwareMap);
    }
}
