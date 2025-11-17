package org.firstinspires.ftc.teamcode.teleop.mainRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;

@Config
public abstract class TeleOpMethods extends RobotBaseHwMap {

    public static final double PURPLE_LED_COLOR_VALUE = 0.91; // normal
    public static final double BLUE_LED_COLOR_VALUE = 0.65;   // shooting
    public static final double RED_LED_COLOR_VALUE = -0.77;   // intake

    boolean requireCameraLockToShoot = false;
    boolean lockPrevPressed = false;

    @Override
    public void init() {super.init();}

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
        boolean xPressed = gamepad1.x;
        intake.runIntake(xPressed);
        transfer.runTransferWithAutomaticStop(xPressed);
    }

    public void runTransfer() {
        boolean holdToEject = gamepad1.left_trigger > 0.5;

        if (holdToEject) {
            if (requireCameraLockToShoot) {
                if (vision.alignedForShot() && shooter.isAtShootingSpeed()) {
                    transfer.runTransferIn();
                }
                else {
                    transfer.runTransferStop();
                }
            }
            else {
                if (shooter.isAtShootingSpeed()) {
                    transfer.runTransferIn();
                }
                else {
                    transfer.runTransferStop();
                }
            }
        }
        else {
            transfer.runTransferStop();
        }
    }

    public void runOuttake() {
        if (requireCameraLockToShoot) {
            if (vision.isDetectingAGoalTag()) {
                shooter.runOuttake(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, telemetry, vision.getTagHorizontalDistance());
                shooter.dynamicallyUpdateHoodPosition(vision.getTagHorizontalDistance());
            }
        }
        else {
            int defaultDistance = 40; // TODO midshot inches
            shooter.runOuttake(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, telemetry, 40);
            shooter.dynamicallyUpdateHoodPosition(defaultDistance);
        }
    }

    @SuppressLint("DefaultLocale")
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("Drive Mode: ", drive.getDriveMode());
        telemetry.addData("Is Tag detected: ", vision.isDetectingAGoalTag());
        telemetry.addData("Sequence: " , vision.getSequence());
    }
}
