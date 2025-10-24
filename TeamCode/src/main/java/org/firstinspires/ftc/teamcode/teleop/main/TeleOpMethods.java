package org.firstinspires.ftc.teamcode.teleop.main;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {

    boolean lockPrevPressed = false;

    @Override
    public void init() {super.init();}
    // put robot methods you will run in the teleOp here
    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x * 1.1; // ! ADJUST TURNING SENSITIVITY HERE ! //
        boolean yButton = gamepad1.y;

        boolean lockPressed = gamepad1.right_stick_button;

        if (lockPressed && !lockPrevPressed) {
            if (drive.getDriveMode() == TeleOpMecanumDrive.DriveMode.MANUAL) {
                drive.setDriveMode(TeleOpMecanumDrive.DriveMode.LOCKED_ON);
            } else {
                drive.setDriveMode(TeleOpMecanumDrive.DriveMode.MANUAL);
            }
            lockPrevPressed = true;
        }

        if (!lockPressed && lockPrevPressed) {
            lockPrevPressed = false;
        }

        if (drive.getDriveMode() == TeleOpMecanumDrive.DriveMode.LOCKED_ON && vision.isDetectingAGoalTag())
        {
            drive.runAutoAlignToTag(Math.toRadians(vision.getTagBearing()), rb, lb, leftY, leftX);
        }
        else {
            drive.runManualMecanumDrive(rb, lb, leftY, leftX, rightX, yButton);
        }
    }

    public void runVision() {
        vision.scanGoalTagSequence();
    }
    @SuppressLint("DefaultLocale")
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("Drive Mode: ", drive.getDriveMode());
        telemetry.addData("Is Tag detected: ", vision.isDetectingAGoalTag());
//        telemetry.addData("imu heading: ", String.format("%.2f", drive.pinpoint.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("Tag Bearing: ", vision.getTagBearing());
        telemetry.addData("Sequence: " , vision.getSequence());
    }
}
