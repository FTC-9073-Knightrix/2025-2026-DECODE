package org.firstinspires.ftc.teamcode.teleop.mainRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision.AprilTagEnums;

@Config
public abstract class TeleOpMethods extends RobotBaseHwMap {
    public enum AllianceColor {
        RED,
        BLUE
    }
    public AllianceColor allianceColor;

    public static class GoalCoords {

        // coords based on pedro pathing field coordinate system for distance calculations
        public static final double RedGoalXPEDRO = 144;
        public static final double RedGoalYPEDRO = 144;
        public static final double BlueGoalXPEDRO = 0;
        public static final double BlueGoalYPEDRO = 144;

        // don't aim exactly at the corners of the goals, aim a bit more towards the center
        public static final double RedGoalXPedroForAiming = 141;
        public static final double RedGoalYPedroForAiming = 141;
        public static final double BlueGoalXPedroForAiming = 3;
        public static final double BlueGoalYPedroForAiming = 141;
    }

    protected enum AimingMethod {
        ODOMETRY,
        CAMERA,
        TESTING,
        MANUAL_ADJUST
    }
    protected AimingMethod robotAimingMethod = AimingMethod.TESTING; // default on odometry

    // Endgame rumble
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
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
        loopTime.reset();
    }

    public void toggleCameraRequirement() {
        // toggle the tracking method between odometry and web camera
        boolean turnOnOdometryButton = gamepad1.dpad_left;
        boolean turnOnCameraButton = gamepad1.dpad_right;
        if (turnOnCameraButton) {
            robotAimingMethod = AimingMethod.CAMERA;
        }
        else if (turnOnOdometryButton) {
            robotAimingMethod = AimingMethod.ODOMETRY;
        }
    }

    public void runTurret() {
        boolean holdAim = gamepad1.left_trigger > 0.5;
        if (robotAimingMethod == AimingMethod.CAMERA) {
            double offsetDegrees = vision.getGoalTagBearing();
            turret.run(offsetDegrees, gamepad1, telemetry);
        }
        else if (robotAimingMethod == AimingMethod.ODOMETRY) {
            switch (allianceColor)  {
                case RED:
                    double offsetRedDegrees = Math.toDegrees(drive.getRobotOdoHeadingOffset(GoalCoords.RedGoalXPedroForAiming, GoalCoords.RedGoalYPedroForAiming));
                    turret.run(offsetRedDegrees, gamepad1, telemetry);
                    break;
            }
        }
    }

    public void runToggledDrive() {
        drive.follower.update(); // update pedro follower every loop

        boolean rb = gamepad1.right_bumper;

        double leftY = gamepad1.left_stick_y;
        double leftX = -gamepad1.left_stick_x;
        double rightX = -gamepad1.right_stick_x * 0.7;

        boolean lockTrigger = gamepad1.left_trigger > 0.5;
        boolean resetHeadingButton = gamepad1.y;
        boolean resetPosButton = gamepad1.left_stick_button;
        boolean resetPosInFarZoneButton = gamepad1.right_stick_button;
        boolean resetPosInClozeZoneButton = gamepad1.dpad_up;
        boolean toggleDriveModeButton = gamepad1.right_stick_button;

        if (lockTrigger && robotAimingMethod == AimingMethod.ODOMETRY) {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.ODOMETRY_LOCKED_ON);
        }
        else if (lockTrigger && robotAimingMethod == AimingMethod.CAMERA) {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.CAMERA_LOCKED_ON);
        } else {
            drive.setDriveMode(TeleOpMecanumDrive.DriveMode.MANUAL);
        }

        // RUN DIFFERENT DRIVE MODES BASED ON WHAT THE CURRENT DRIVE MODE IS
        switch (drive.getDriveMode()) {
            case ODOMETRY_LOCKED_ON:
                switch (allianceColor)  {
                    case RED:
                        double offsetRadRed = drive.getRobotOdoHeadingOffset(GoalCoords.RedGoalXPedroForAiming, GoalCoords.RedGoalYPedroForAiming);
                        drive.runAutoAlignToTag(offsetRadRed, rb, leftY, leftX, allianceColor);

                        if (Math.abs(Math.toDegrees(offsetRadRed)) < 2.0) {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        else {
                            lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        break;
                    case BLUE:
                        double offsetRadBlue = drive.getRobotOdoHeadingOffset(GoalCoords.BlueGoalXPedroForAiming, GoalCoords.BlueGoalYPedroForAiming);
                        drive.runAutoAlignToTag(offsetRadBlue, rb, leftY, leftX, allianceColor);

                        if (Math.abs(Math.toDegrees(offsetRadBlue)) < 2.0) {
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

                    drive.runAutoAlignToTag(Math.toRadians(vision.getGoalTagBearing() + offsetDegrees), rb, leftY, leftX, allianceColor);

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
                    drive.runManualMecanumDrive(gamepad1, allianceColor);
                    lights.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                break;
            case MANUAL:
                drive.runManualMecanumDrive(gamepad1, allianceColor);
                drive.toggleRobotCentric(toggleDriveModeButton);
                lights.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                break;
        }
    }

    // gonna use a manual drive now because the turret will
    // aim towards the goal, not the drive train
    public void runManualDrive() {
        drive.follower.update(); // update pedro follower every loop

        boolean toggleDriveModeButton = gamepad1.right_stick_button;

        drive.runManualMecanumDrive(gamepad1, allianceColor);
        drive.toggleRobotCentric(toggleDriveModeButton);
        lights.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    public void runIntake() {
        boolean forceEject = gamepad1.b;
        boolean toggleButton = gamepad1.left_bumper;
        intake.runIntake(toggleButton, forceEject);
    }

    public void runTransfer() {
        boolean holdToShootTrigger = gamepad1.right_trigger > 0.5;
        if (holdToShootTrigger) {
            double power = 0;
            if (shooter.isSingleAtShootingSpeed()) {
                transfer.openGate();
                power = -1.0;
            }
            else if (transfer.gateIsOpen) {
                power = -0.5;
            }
            intake.intakeMotor.setPower(power);
            intake.intakeMotor2.setPower(power);
        }
        else {
            transfer.closeGate();
        }
    }

    public void runOuttake() {
        // if the robot aiming method is camera, use vision horizontal tag distance to aim
        // if the robot aiming method is odometry, use odometry distance formula to aim
        // if the robot aiming method is manual adjust, use gamepad 2 dpad to adjust
        switch (robotAimingMethod) {
            case CAMERA:
                shooter.runCameraShots(gamepad1, telemetry, vision.getGoalTagHorizontalDistance());
                break;
            case ODOMETRY:
                double targetX = (allianceColor == AllianceColor.RED) ? GoalCoords.RedGoalXPEDRO : GoalCoords.BlueGoalXPEDRO;
                double targetY = (allianceColor == AllianceColor.RED) ? GoalCoords.RedGoalYPEDRO : GoalCoords.BlueGoalYPEDRO;
                double distance = drive.getOdometryDistanceFromGoal(targetX, targetY);
                shooter.runOdometryShots(gamepad1, telemetry, distance);
                break;
            case TESTING:
                shooter.testOuttake(gamepad1.a, telemetry, gamepad2);
                break;
        }
    }

    @SuppressLint("DefaultLocale")
    public void displayTelemetry() {
        telemetry.addData("loopTime", loopTime.milliseconds());
        telemetry.addData("Aiming method: ", robotAimingMethod);
        telemetry.addData("alliance", allianceColor);
        telemetry.addData("Is Tag detected: ", vision.isDetectingAGoalTag());
        double intakeTicksPerSecond = intake.intakeMotor.getVelocity();
        double intakeRPM = (intakeTicksPerSecond / 145.1) * 60.0;
        telemetry.addData("intake velocity (RPM)", String.format("%.1f", intakeRPM));
        //robot pose
        telemetry.addData("Pose: ", String.valueOf(drive.follower.getPose()));
        telemetry.addData("Distance:", String.format("%.1f", drive.getOdometryDistanceFromGoal(GoalCoords.RedGoalXPEDRO, GoalCoords.RedGoalYPEDRO)));
        telemetry.update();
    }
}
