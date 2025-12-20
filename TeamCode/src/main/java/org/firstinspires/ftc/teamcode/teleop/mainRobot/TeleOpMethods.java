package org.firstinspires.ftc.teamcode.teleop.mainRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.RGBLights;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision.AprilTagEnums;

@Config
public abstract class TeleOpMethods extends RobotBaseHwMap {
    public enum AllianceColor {
        RED,
        BLUE
    }
    public AllianceColor allianceColor;

    public class GoalCoords {
        public static final double RedGoalX = -72.0;
        public static final double RedGoalY = -72.0;
        public static final double BlueGoalX = -72.0;
        public static final double BlueGoalY = 72.0;
    }

    boolean requireCameraToShoot = true;
    boolean lastCameraTogglePressed = false;
    ElapsedTime gameTime = new ElapsedTime();
    boolean reachedEndGame = false;

    @Override
    public void init() {super.init();}

    public void rumbleGamePads() {
        // rumble the gamepads if endgame is reached (the last 20 seconds)
        if (gameTime.seconds() > 100 && !reachedEndGame) {
            reachedEndGame = true;
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }
    }

    public void toggleCameraRequirement() {
        boolean cameraTogglePressed = gamepad2.y;
        if (cameraTogglePressed && !lastCameraTogglePressed) {
            requireCameraToShoot = !requireCameraToShoot;
        }
        lastCameraTogglePressed = cameraTogglePressed;
    }

    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;

        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        boolean lockTrigger = gamepad1.left_trigger > 0.5;
        boolean resetHeadingButton = gamepad1.y;
        boolean toggleDriveModeButton = gamepad1.right_stick_button;

        if (lockTrigger && !requireCameraToShoot) {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.ODOMETRY_LOCKED_ON);
        }
        else if (lockTrigger) {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.CAMERA_LOCKED_ON);
        }else {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.MANUAL);
        }

        // RUN DIFFERENT DRIVE MODES BASED ON WHAT THE CURRENT DRIVE MODE IS
        switch (drive.getDriveMode()) {
            case ODOMETRY_LOCKED_ON:
                switch (allianceColor)  {
                    case RED:
                        double offsetRadRed = drive.getRobotOdoHeadingOffset(GoalCoords.RedGoalX, GoalCoords.RedGoalY);
                        drive.runAutoAlignToTag(offsetRadRed, rb, leftY, leftX);

                        if (Math.toDegrees(offsetRadRed) < 1.5) {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        else {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        break;
                    case BLUE:
                        double offsetRadBlue = drive.getRobotOdoHeadingOffset(GoalCoords.BlueGoalX, GoalCoords.BlueGoalY);
                        drive.runAutoAlignToTag(offsetRadBlue, rb, leftY, leftX);

                        if (Math.toDegrees(offsetRadBlue) < 1.5) {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        else {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        break;
                }
                break;
            case CAMERA_LOCKED_ON:
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

                    drive.runAutoAlignToTag(Math.toRadians(vision.getGoalTagBearing() + offsetDegrees), rb, leftY, leftX);

                    // SET LIGHTS TO GREEN IF THE CAMERA IS LOCKED ON
                    // try to align with offset (the negative of the offsetDegrees)
                    if (vision.alignedForShot(-offsetDegrees)) {
                        lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    else {
                        lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                }
                else {
                    drive.runManualMecanumDrive(rb, leftY, leftX, rightX, resetHeadingButton);
                    // red color because camera is not detecting tag
                    lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                break;
            case MANUAL:
                drive.runManualMecanumDrive(rb, leftY, leftX, rightX, resetHeadingButton);
                drive.toggleRobotCentric(toggleDriveModeButton);
                lights.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                break;
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
//        telemetry.addData("Tag Horizontal Distance (in): " , String.format("%.2f", vision.getGoalTagHorizontalDistance()));
//        telemetry.addData("Tag Bearing:", String.format("%.2f", vision.getGoalTagBearing()));
//        telemetry.addData("distance sensor: (CM)" , transfer.transferDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("offset rad", drive.getRobotOdoHeadingOffset(-72, 72));
        telemetry.addData("robot pose", drive.pinpoint.getPosition());
//        telemetry.addData("transfer active: " , transfer.transferActive);
        telemetry.update();
    }
}
