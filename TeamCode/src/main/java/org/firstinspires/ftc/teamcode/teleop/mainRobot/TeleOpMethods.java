package org.firstinspires.ftc.teamcode.teleop.mainRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.RGBLights;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision.AprilTagEnums;

@Config
public abstract class TeleOpMethods extends RobotBaseHwMap {
    boolean requireCameraLockToShoot = false;

    @Override
    public void init() {super.init();}

    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        boolean lockTrigger = gamepad1.left_trigger > 0.5;
        boolean resetHeadingButton = gamepad1.y;
        boolean toggleDriveModeButton = gamepad1.right_stick_button;

        if (lockTrigger) {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.LOCKED_ON);
        } else {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.MANUAL);
        }

        if (drive.getDriveMode() == TeleOpMecanumDrive.DriveMode.LOCKED_ON && vision.isDetectingAGoalTag())
        {
            double offsetDegrees = 0.0;
            if (vision.getGoalTagHorizontalDistance() < 100.0) {
                offsetDegrees = 0.0;
            }
            else if (vision.getDetectedTagId() == AprilTagEnums.RED_GOAL.getId()) {
                offsetDegrees = -3.0;
            }
            else if (vision.getDetectedTagId() == AprilTagEnums.BLUE_GOAL.getId()) {
                offsetDegrees = 3.0;
            }

            drive.runAutoAlignToTag(Math.toRadians(vision.getGoalTagBearing() + offsetDegrees), rb, lb, leftY, leftX);
            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else {
            drive.runManualMecanumDrive(rb, lb, leftY, leftX, rightX, resetHeadingButton);
            drive.toggleRobotCentric(toggleDriveModeButton);
            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    public void runVision() {
        vision.scanMotifTagSequence();
    }

    public void runIntake() {
        boolean xPressed = gamepad1.x;
        intake.runIntake(xPressed);
//        transfer.runTransferWithAutomaticStop(xPressed);
    }

    public void runTransfer() {
        boolean holdToShoot = gamepad1.right_trigger > 0.5;

        if (holdToShoot) {
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
                } else {
                    transfer.runTransferStop();
                }
            }
        }
        else if (gamepad1.dpad_up) {
            // force transfer out
            transfer.runTransferOut();
        }
        else if (gamepad1.left_bumper) {
            // force transfer in
            transfer.runTransferIn();
        }
        else {
            transfer.runTransferStop();
        }
    }

    public void runOuttake() {
        if (requireCameraLockToShoot) {
            if (vision.isDetectingAGoalTag()) {
                shooter.runOuttake(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, telemetry, vision.getGoalTagHorizontalDistance());
                shooter.dynamicallyUpdateHoodPosition(vision.getGoalTagHorizontalDistance());
            }
        }
        else {
            shooter.runOuttake(gamepad1.a, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, telemetry, vision.getGoalTagHorizontalDistance());
            shooter.dynamicallyUpdateHoodPosition(vision.getGoalTagHorizontalDistance());
        }
    }

    @SuppressLint("DefaultLocale")
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("Drive Mode: ", drive.getDriveMode());
        telemetry.addData("Is Tag detected: ", vision.isDetectingAGoalTag());
        telemetry.addData("Sequence: " , vision.getMotif());
        telemetry.addData("Tag Horizontal Distance (in): " , String.format("%.2f", vision.getGoalTagHorizontalDistance()));
        telemetry.addData("Tag Bearing:", String.format("%.2f", vision.getGoalTagBearing()));
        telemetry.addData("distance sensor: (CM)" , transfer.transferDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("transfer active: " , transfer.transferActive);
        telemetry.update();
    }
}
