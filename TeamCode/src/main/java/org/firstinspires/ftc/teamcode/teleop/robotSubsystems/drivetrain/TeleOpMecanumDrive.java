package org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.mainRobot.TeleOpMethods.AllianceColor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.RobotStaticVariables.END_OF_AUTO_POSITION;

public class TeleOpMecanumDrive {
    public enum DriveMode {
        MANUAL,
        CAMERA_LOCKED_ON,
        ODOMETRY_LOCKED_ON
    }

    private DriveMode driveMode = DriveMode.MANUAL;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public IMU rev_imu;
    public YawPitchRollAngles orientation;
    public GoBildaPinpointDriver pinpoint;
    public Follower follower;

    public double finalSlowMode = 0.0;
    public final double driveSpeed = 0.66;
    public final double fastSpeed = 1.0;
    public final double slowSpeed = 0.30;

    boolean robotCentric;
    private boolean toggleRobotCentricButtonPrevPressed = false;

    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backRightMotor = hwMap.get(DcMotor.class, "rightBack");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        follower = Constants.createFollower(hwMap);
        rev_imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rev_imu.initialize(new IMU.Parameters(RevOrientation));
        robotCentric = false;

        follower.setPose(END_OF_AUTO_POSITION);
        driveTimer.reset();
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void runManualMecanumDrive(Gamepad gamepad1, AllianceColor allianceColor) {
        // boolean rb, double y, double x, double rx, boolean resetHeadingButton, boolean resetPosButton, boolean resetPosInCloseZone, boolean resetPosInFarZone,
//        boolean rb = gamepad1.right_bumper;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        rx = Math.signum(rx) * rx * rx; // square the turning input for finer control

        boolean resetHeadingButton = gamepad1.y;
        boolean resetPosButton = gamepad1.left_stick_button;

        finalSlowMode = fastSpeed;

        if (resetHeadingButton) {
            rev_imu.resetYaw();
//            pinpoint.recalibrateIMU();
        }

        // for testing, reset to known positions
        if (resetPosButton) {
            if (allianceColor == AllianceColor.BLUE) {
                follower.setPose(new Pose(55.83673469387756, 8.326530612244904, Math.toRadians(90)));
            } else {
                follower.setPose(new Pose(144 - 55.83673469387756, 8.326530612244904, Math.toRadians(-90)));
            }
        }

        orientation = rev_imu.getRobotYawPitchRollAngles();

        // CHANGE BETWEEN PP AND IMU U CHANGE BOTHEADING VARIABLE
        double botHeading = rev_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double rotX, rotY;
        if (robotCentric) {
            rotX = x;
            rotY = y;
        }
        else {
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }

        // changed the scaling from quadratic to cubic for better control
        if (finalSlowMode == fastSpeed) {
            rotX = rotX * rotX * rotX;
            rotY = rotY * rotY * rotY;
        }

        // scale rotX to speed up turning
        rotX = rotX * 1.3;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // clip all powers
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);

        this.frontLeftMotor.setPower(frontLeftPower * finalSlowMode);
        this.backLeftMotor.setPower(backLeftPower * finalSlowMode);
        this.frontRightMotor.setPower(frontRightPower * finalSlowMode);
        this.backRightMotor.setPower(backRightPower * finalSlowMode);
    }

    public void toggleRobotCentric(boolean toggleButtonPressed) {
        if (toggleButtonPressed && !toggleRobotCentricButtonPrevPressed) {
            robotCentric = !robotCentric;
        }
        toggleRobotCentricButtonPrevPressed = toggleButtonPressed;
    }

    // returns the offset between the heading of the robot and the tag
    // in RADIANS
    public double getRobotOdoHeadingOffset(double targetGoalX, double targetGoalY) {
        // pedro pathing pose (in inches and radians)
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = currentPose.getHeading();

        // signed differences
        double dx = targetGoalX - robotX;
        double dy = targetGoalY - robotY;

        // angle from robot to target in field coords
        double targetAngle = Math.atan2(dy, dx);

        // raw difference and robust normalization to [-PI, PI]
        double offset = targetAngle - robotHeading;

        return normalizeAngle(offset); // radians, in range (-PI, PI]
    }

    public static double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    public double getOdometryDistanceFromGoal(double targetGoalX, double targetGoalY) {
        // using pedro system
        Pose currentPose = follower.getPose();
        double deltaX = targetGoalX   - currentPose.getX();
        double deltaY = targetGoalY - currentPose.getY();

        return Math.hypot(deltaX, deltaY);
    }

    private double lastBearingError = 0.0;
    private double integralSum = 0.0;
    ElapsedTime driveTimer = new ElapsedTime();
    public void runAutoAlignToTag(double bearingOffsetRad, boolean rb, double y, double x, AllianceColor allianceColor) {
        // PID coefficients
        double kP = 0.75;
        double kI = 0.2;
        double kD = 0.02;

        double maxPower = 1.0; // maximum turn power
        double alignmentThreshold = 0.01; // radians, adjust as needed
        double minPower = 0.08; // minimum power to overcome static friction
        double turnPower = 0.0;

        if (Math.abs(bearingOffsetRad) > alignmentThreshold) {
            double dt = Math.max(driveTimer.seconds(), 0.001); // guard against zero time interval

            // Accumulate error for integral term
            integralSum += bearingOffsetRad * dt;

            // Prevent integral windup
            double maxIntegral = 0.3;
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

            double derivative = (bearingOffsetRad - lastBearingError) / dt;
            derivative = Range.clip(derivative, -5.0, 5.0);

            turnPower = (-kP * bearingOffsetRad) + (-kI * integralSum) + (-kD * derivative);

            // Add minimum power to overcome static friction
            if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            turnPower = Range.clip(turnPower, -maxPower, maxPower);
        } else {
            // Reset integral when aligned
            integralSum = 0.0;
        }

        lastBearingError = bearingOffsetRad;
        driveTimer.reset();

        // The Driver can still translate while auto-aligning, but cannot manually rotate
//        runManualMecanumDrive(rb, y, x, turnPower, false, false, false, false, allianceColor);
    }
}
