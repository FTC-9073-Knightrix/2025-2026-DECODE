package org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TeleOpMecanumDrive {
    public enum DriveMode {
        MANUAL,
        LOCKED_ON
    }
    private DriveMode driveMode = DriveMode.MANUAL;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public IMU rev_imu;
    public YawPitchRollAngles orientation;
    public GoBildaPinpointDriver pinpoint;

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

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rev_imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rev_imu.initialize(new IMU.Parameters(RevOrientation));

        robotCentric = false;
        driveTimer.reset();
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void runManualMecanumDrive(boolean rb, boolean lb, double y, double x, double rx, boolean resetHeadingButton) {
        // Only update the heading because that is all you need in Teleop
//        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        if (rb) {
            finalSlowMode = slowSpeed;
        } else {
            finalSlowMode = fastSpeed;
        }

        if (resetHeadingButton) {
            rev_imu.resetYaw();
//            pinpoint.resetPosAndIMU();
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

        // If we're in fast mode, apply a quadratic (signed square) scaling to
        // translational inputs so small joystick deflections are finer while
        // full deflections still reach maximum speed.
        if (finalSlowMode == fastSpeed) {
            rotX = Math.signum(rotX) * rotX * rotX;
            rotY = Math.signum(rotY) * rotY * rotY;
        }

        // scale rotX to speed up turning
        rotX = rotX * 1.2;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        this.frontLeftMotor.setPower(frontLeftPower * finalSlowMode);
        this.backLeftMotor.setPower(backLeftPower * finalSlowMode);
        this.frontRightMotor.setPower(frontRightPower * finalSlowMode);
        this.backRightMotor.setPower(backRightPower * finalSlowMode);

        driveTimer.reset(); // reset timer when in manual mode so derivative term is accurate when switching to auto-align
    }

    public void toggleRobotCentric(boolean toggleButtonPressed) {
        if (toggleButtonPressed && !toggleRobotCentricButtonPrevPressed) {
            robotCentric = !robotCentric;
        }
        toggleRobotCentricButtonPrevPressed = toggleButtonPressed;
    }

    private double lastBearingError = 0.0;
    private double integralSum = 0.0;
    ElapsedTime driveTimer = new ElapsedTime();
    public void runAutoAlignToTag(double bearingOffsetRad, boolean rb, boolean lb, double y, double x) {
        // PID coefficients
        double kP = 0.75;
        double kI = 0.15; // Integral coefficient - helps overcome static friction
        double kD = 0.01; // TODO TUNE

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
        runManualMecanumDrive(rb, lb, y, x, turnPower, false);
    }
}
