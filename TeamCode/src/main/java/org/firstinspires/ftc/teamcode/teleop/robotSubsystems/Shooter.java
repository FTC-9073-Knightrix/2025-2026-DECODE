package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    Servo hoodServo;
    DcMotorEx outtakeMotor;


    private final double FAR_SHOT_VELOCITY_TICKS = -1500.0;
    private final double MID_FAR_SHOT_VELOCITY_TICKS = -1250;
    private final double MID_SHOT_VELOCITY_TICKS = -1150.0;
    private final double NEAR_SHOT_VELOCITY_TICKS = -1050.0;
    private final double ACCEPTABLE_VELOCITY_ERROR_TICKS = 125.0;

    public double targetVelocityTicks = MID_SHOT_VELOCITY_TICKS; // start off at mid shot velocity

    private final int FAR_INCHES = 85;
    private final int MID_INCHES = 45;
    private final int MID_FAR_INCHES = 55;

    private final double CLOSE_SHOT_HOOD = 0.85;
    private final double MID_SHOT_HOOD = 0.60;
    private final double FAR_SHOT_HOOD = 0.45;
    private double hoodPosition = 0.85;

    private boolean outtakeOn = false; // start the match with outtake on
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // PIDF tuning resources: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    // After kV is set, tune kP to minimize error, use small increases
    private final double kP = 29;
    private final double kI = 0.9;
    private final double kD = 0.0;
    private final double kF = 0.7;


    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF); // we use only kP and kF here
        hoodServo.setPosition(hoodPosition);
    }

    public void runTestOuttake(boolean a, boolean dpad_left, boolean dpad_right,
                               boolean dpad_up, boolean dpad_down,
                               Telemetry telemetry, double horizontalDistanceToGoalInches) {

        // update target velocity based on distance to goal if needed
//        updateShooterVelocityByDistance(horizontalDistanceToGoalInches);

        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = a;

        // Servo adjust
        if (dpad_left && !lastDpadLeft) {
            hoodPosition = Math.min(1.0, hoodPosition + 0.05);
            hoodServo.setPosition(hoodPosition);
        }
        lastDpadLeft = dpad_left;

        if (dpad_right && !lastDpadRight) {
            hoodPosition = Math.max(0.0, hoodPosition - 0.05);
            hoodServo.setPosition(hoodPosition);
        }
        lastDpadRight = dpad_right;

        // Adjust velocity
        if (dpad_up && !lastDpadUp) {
            targetVelocityTicks -= 20;
            if (targetVelocityTicks < -2000) targetVelocityTicks = -2000;
        }
        lastDpadUp = dpad_up;

        if (dpad_down && !lastDpadDown) {
            targetVelocityTicks += 25;
        }
        lastDpadDown = dpad_down;

        // Apply velocity control
        if (outtakeOn) {
            outtakeMotor.setVelocity(targetVelocityTicks);
        } else {
            outtakeMotor.setVelocity(0);
        }

        // Telemetry
        double ticksPerSecond = outtakeMotor.getVelocity();

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
    }

    public void runManualOuttake(boolean a, boolean dpad_left, boolean dpad_right,
                           boolean dpad_up, boolean dpad_down,
                           Telemetry telemetry) {

        // update target velocity based on distance to goal if needed
        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = a;

        // Close shot
        if (dpad_down) {
            targetVelocityTicks = NEAR_SHOT_VELOCITY_TICKS;
            hoodPosition = CLOSE_SHOT_HOOD;
        }
        else if (dpad_left) { // Mid shot
            targetVelocityTicks = MID_SHOT_VELOCITY_TICKS;
            hoodPosition = MID_SHOT_HOOD;
        }
        else if (dpad_right) { // Mid-far shot
            targetVelocityTicks = MID_FAR_SHOT_VELOCITY_TICKS;
            hoodPosition = FAR_SHOT_HOOD;
        }
        else if (dpad_up) { // Far shot
            targetVelocityTicks = FAR_SHOT_VELOCITY_TICKS;
            hoodPosition = FAR_SHOT_HOOD;
        }

        // Apply velocity control
        if (outtakeOn) {
            outtakeMotor.setVelocity(targetVelocityTicks);
        } else {
            outtakeMotor.setVelocity(0);
        }

        hoodPosition = Range.clip(hoodPosition, 0.0, 1.0);
        hoodServo.setPosition(hoodPosition);

        // Telemetry
        double ticksPerSecond = outtakeMotor.getVelocity();

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
    }

    public void runDynamicOuttake(boolean a, boolean forceFarShot, Telemetry telemetry, double horizontalDistanceToGoalInches) {

        // update target velocity based on distance to goal if needed
        updateShooterVelocityByDistance(horizontalDistanceToGoalInches);
        dynamicallyUpdateHoodPosition(horizontalDistanceToGoalInches);
        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = a;

        // Apply velocity control
        if (outtakeOn) {
            if (forceFarShot) {
                targetVelocityTicks = FAR_SHOT_VELOCITY_TICKS;
                hoodPosition = FAR_SHOT_HOOD;
            }
            outtakeMotor.setVelocity(targetVelocityTicks);
        } else {
            outtakeMotor.setVelocity(0);
        }

        // Telemetry
        double ticksPerSecond = outtakeMotor.getVelocity();

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
    }

    public void runDynamicOdometryOuttake(boolean a, Telemetry telemetry, double horizontalDistanceToGoalInches) {
        // update target velocity based on distance to goal if needed
        updateShooterVelocityByOdometryDistance(horizontalDistanceToGoalInches);
        dynamicallyUpdateHoodPositionByOdometry(horizontalDistanceToGoalInches);
//        updateShooterVelocityByDistance(horizontalDistanceToGoalInches);
//        dynamicallyUpdateHoodPosition(horizontalDistanceToGoalInches);
        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = a;

        // Apply velocity control
        if (outtakeOn) {
            outtakeMotor.setVelocity(targetVelocityTicks);
        } else {
            outtakeMotor.setVelocity(0);
        }

        // Telemetry
        double ticksPerSecond = outtakeMotor.getVelocity();

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
    }
    public void dynamicallyUpdateHoodPosition(double horizontalDistanceToGoalInches) {
        if (horizontalDistanceToGoalInches < 0) {
            // No valid distance, do not update hood position
            return;
        }

        double x = horizontalDistanceToGoalInches;
        // TODO Create a regression here based on empirical data points (desmos.com)
//        double targetHoodPosition = 0.0000025 * x * x - 0.0015 * x + 0.85; // a filler regression curve
//        double deltaPos = targetHoodPosition - hoodPosition;
//
//        if (Math.abs(deltaPos) < 0.05) {
//            hoodPosition = targetHoodPosition;
//        } else {
//            double adjustment = deltaPos > 0 ? 0.05 : -0.05;
//            hoodPosition += adjustment;
//        }

        if (horizontalDistanceToGoalInches > FAR_INCHES) {
            hoodPosition = FAR_SHOT_HOOD;
        }
        else if (horizontalDistanceToGoalInches > MID_FAR_INCHES) {
            hoodPosition = FAR_SHOT_HOOD;
        }
        else if (horizontalDistanceToGoalInches > MID_INCHES) {
            hoodPosition = MID_SHOT_HOOD;
        }
        else {
            hoodPosition = CLOSE_SHOT_HOOD;
        }
        // Clamp the hood position to valid servo range [0.0, 1.0]
        hoodPosition = Range.clip(hoodPosition, 0.0, 1.0);

        hoodServo.setPosition(hoodPosition);
    }

    public void updateShooterVelocityByDistance(double horizontalDistanceToGoalInches) {
        if (horizontalDistanceToGoalInches < 0) {
            // No valid distance, do not update velocity
            return;
        }

        double distance = horizontalDistanceToGoalInches;

        if (distance > FAR_INCHES) { // FAR SHOT
            targetVelocityTicks = FAR_SHOT_VELOCITY_TICKS;
        } else if (distance > MID_FAR_INCHES) {
            targetVelocityTicks = MID_FAR_SHOT_VELOCITY_TICKS;
        } else if (distance > MID_INCHES) { // MID SHOT
            targetVelocityTicks = MID_SHOT_VELOCITY_TICKS;
        } else { // NEAR SHOT
            targetVelocityTicks = NEAR_SHOT_VELOCITY_TICKS;
        }
    }

    public void updateShooterVelocityByOdometryDistance(double x) {
        if (x < 0) {
            // No valid distance, do not update velocity
            return;
        }

        // regression from desmos of ticks plotted vs distance
        // y = -6.47098x-621.32676
        targetVelocityTicks = -6.47098 * x - 621.32676;
        targetVelocityTicks = Range.clip(targetVelocityTicks, -1600, -1000);
    }

    public void dynamicallyUpdateHoodPositionByOdometry(double x) {
        // the hood will be a function of the shooter velocity
        // to allow for a velocity-based hood for rapid firing

        // have a separate regression to handle for far shot because not modeled well experimentally
        double hoodPos;
        if (x < 120) {
            // y=0.185111\cdot\sin\left(0.00671479x+1.80341\right)+0.582581
            // close shot
            hoodPos = 0.185111 * Math.sin(0.00671479 * x + 1.80341) + 0.582581;
        }
        else {
            // far shot
            // y=0.0000126263x^{2}+0.0390152x+30.57374
            hoodPos = 0.0000126263 * x * x + 0.0390152 * x + 30.57374;
        }

        hoodPosition = Range.clip(hoodPos, 0.4, 0.75);
        hoodServo.setPosition(hoodPosition);
    }
    public boolean isAtShootingSpeed() {
        double currentVelocity = outtakeMotor.getVelocity();
        double velocityError = Math.abs(targetVelocityTicks - currentVelocity);

        return velocityError <= ACCEPTABLE_VELOCITY_ERROR_TICKS;
    }
}
