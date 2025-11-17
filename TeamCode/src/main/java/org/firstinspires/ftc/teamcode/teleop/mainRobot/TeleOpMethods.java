package org.firstinspires.ftc.teamcode.teleop.mainRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;

@Config
public abstract class TeleOpMethods extends RobotBaseHwMap {

    boolean lockPrevPressed = false;
    public static final double PURPLE = 0.91; // normal
    public static final double BLUE = 0.65;   // shooting
    public static final double RED = -0.77;   // intake
    boolean requireCameraLockToShoot = false;

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

    public void runIntake() {
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        intake.runIntake(leftTriggerPressed);
    }

    public void runTransfer() {
        if (requireCameraLockToShoot) {
            if (gamepad1.right_trigger > 0.5 && vision.alignedForShot() && shooter.isAtShootingSpeed()) {
                transfer.runTransferFeed();
            }
        }
        else if (gamepad1.right_trigger > 0.5 && shooter.isAtShootingSpeed()) {
            transfer.runTransferFeed();
        } else {
            transfer.runTransferStop();
        }

    }
    public void runOuttake() {
        shooter.runOuttake(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, telemetry);
        shooter.dynamicallyUpdateHoodPosition(vision.getTagHorizontalDistance());
    }


    @SuppressLint("DefaultLocale")
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("Drive Mode: ", drive.getDriveMode());
        telemetry.addData("Is Tag detected: ", vision.isDetectingAGoalTag());
        telemetry.addData("Sequence: " , vision.getSequence());
    }
}
