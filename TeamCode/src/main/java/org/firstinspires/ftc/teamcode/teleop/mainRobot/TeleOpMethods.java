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
    boolean requireCameraToShoot = true;
    boolean lastCameraTogglePressed = false;
    @Override
    public void init() {super.init();}

    public void toggleCameraRequirement() {
        boolean cameraTogglePressed = gamepad2.y;
        if (cameraTogglePressed && !lastCameraTogglePressed) {
            requireCameraToShoot = !requireCameraToShoot;
        }
        lastCameraTogglePressed = cameraTogglePressed;

        if (requireCameraToShoot) {
            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else {
            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GRAY);
        }
    }

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

        if (drive.getDriveMode() == TeleOpMecanumDrive.DriveMode.LOCKED_ON)
        {
            if (vision.isDetectingAGoalTag()) {
                double offsetDegrees = 0.0;
                if (vision.getGoalTagHorizontalDistance() < 100.0) {
                    offsetDegrees = 0.0;
                }
                else if (vision.getDetectedTagId() == AprilTagEnums.RED_GOAL.getId()) {
                    offsetDegrees = -3;
                }
                else if (vision.getDetectedTagId() == AprilTagEnums.BLUE_GOAL.getId()) {
                    offsetDegrees = 3;
                }

                drive.runAutoAlignToTag(Math.toRadians(vision.getGoalTagBearing() + offsetDegrees), rb, lb, leftY, leftX);

                // SET LIGHTS TO GREEN IF THE CAMERA IS LOCKED ON
                // try to align with offset (maybe the negative of the offsetDegrees?)
                if (vision.alignedForShot(-offsetDegrees)) {

                    lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                else {
                    lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
            }
            else {
                drive.runManualMecanumDrive(rb, lb, leftY, leftX, rightX, resetHeadingButton);
                // red color because camera is not detecting tag
                lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
        else {
            drive.runManualMecanumDrive(rb, lb, leftY, leftX, rightX, resetHeadingButton);
            drive.toggleRobotCentric(toggleDriveModeButton);
            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }
    }

    public void runIntake() {
        boolean xPressed = gamepad1.x;
        boolean forceEject = gamepad1.b;
        intake.runIntake(xPressed, forceEject);
    }

    public void runTransfer() {
        boolean holdToShoot = gamepad1.right_trigger > 0.5;
        boolean forceTransferForwards = gamepad1.left_bumper;
        boolean forceTransferBackwards = gamepad1.dpad_down;

        if (holdToShoot) {
            if (shooter.isAtShootingSpeed()) {
                transfer.runTransferIn();
            } else {
                transfer.runTransferStop();
            }
        }
        else if (forceTransferBackwards) {
            // force transfer out
            transfer.runTransferOut();
        }
        else if (forceTransferForwards) {
            // force transfer in
            transfer.runTransferForceIn();
        }
        else {
            transfer.runTransferStop();
        }
    }

    public void runOuttake() {
        if (requireCameraToShoot) {
            shooter.runDynamicOuttake(gamepad1.a, gamepad1.left_stick_button, telemetry, vision.getGoalTagHorizontalDistance());
        }
        else {
            shooter.runManualOuttake(gamepad2.a, gamepad2.dpad_left, gamepad2.dpad_right,
                    gamepad2.dpad_up, gamepad2.dpad_down, telemetry);
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
