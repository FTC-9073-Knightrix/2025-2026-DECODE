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
    private boolean rightStickPrevPressed = false;


    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backRightMotor = hwMap.get(DcMotor.class, "rightBack");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rev_imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rev_imu.initialize(new IMU.Parameters(RevOrientation));

        driveTimer.reset();
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    // unused method
    public void runMecanumDrive(boolean rb, boolean lb, double y, double x, double rx, boolean yButton, boolean rightStickPressed, double bearingRadians) {
        // Toggle logic
        if (rightStickPressed && !rightStickPrevPressed) {
            if (driveMode == DriveMode.MANUAL) {
                driveMode = DriveMode.LOCKED_ON;
            } else {
                driveMode = DriveMode.MANUAL;
            }
        }
        rightStickPrevPressed = rightStickPressed;

        // Run drive modes
        switch (driveMode) {
            case MANUAL:
                runManualMecanumDrive(rb, lb, y, x, rx, yButton);
                break;

            case LOCKED_ON:
                runAutoAlignToTag(bearingRadians, rb, lb, y, x);
                break;
        }
    }
    public void runManualMecanumDrive(boolean rb, boolean lb, double y, double x, double rx, boolean yButton) {
        // Only update the heading because that is all you need in Teleop
//        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        //Setting boolean hold
        if(rb) {
            //Slow mode
            finalSlowMode = slowSpeed;
        } else if (lb) {
            //Fast mode
            finalSlowMode = fastSpeed;
        } else {
            //Regular
            finalSlowMode = driveSpeed;
        }

//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x * .8;

        if (yButton) {
            rev_imu.resetYaw();
//            pinpoint.resetPosAndIMU();
        }

        orientation = rev_imu.getRobotYawPitchRollAngles();

        // CHANGE BETWEEN PP AND IMU U CHANGE BOTHEADING VARIABLE
        double botHeading = rev_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

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


    private double lastBearingError = 0.0;
    ElapsedTime driveTimer = new ElapsedTime();

    public void runAutoAlignToTag(double bearingOffsetRad, boolean rb, boolean lb, double y, double x) {
        // proportional and derivate coefficients
        double kP = 0.805;
        double kD = 0.12; // TODO TUNE

        double maxPower = 0.9; // maximum turn power
        double alignmentThreshold = 0.03; // radians, adjust as needed
        double turnPower = 0.0;

        if (Math.abs(bearingOffsetRad) > alignmentThreshold) {
            double dt = Math.max(driveTimer.seconds(), 0.001); // guard against zero time interval
            double derivative = (bearingOffsetRad - lastBearingError) / dt;

            turnPower = (-kP * bearingOffsetRad) + (-kD * derivative);
            turnPower = Range.clip(turnPower, -maxPower, maxPower);

        }
        lastBearingError = bearingOffsetRad;
        driveTimer.reset();

        // The Driver can still translate while auto-aligning, but cannot manually rotate
        runManualMecanumDrive(rb, lb, y, x, turnPower, false);
    }
}
