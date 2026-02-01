package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    Servo hoodServo;
    DcMotorEx outtakeMotor;
    DcMotorEx outtakeMotor2;
    FtcDashboard dashboard;

    private final double FAR_SHOT_VELOCITY_TICKS = 1500.0;
    private final double MID_FAR_SHOT_VELOCITY_TICKS = 1250;
    private final double MID_SHOT_VELOCITY_TICKS = 1150.0;
    private final double NEAR_SHOT_VELOCITY_TICKS = 1050.0;
    private final double ACCEPTABLE_VELOCITY_ERROR_TICKS = 100.0;

    public double targetVelocityTicks = MID_SHOT_VELOCITY_TICKS; // start off at mid shot velocity

    private final int FAR_INCHES = 85;
    private final int MID_INCHES = 45;
    private final int MID_FAR_INCHES = 55;

    private final double CLOSE_SHOT_HOOD = 0.85;
    private final double MID_SHOT_HOOD = 0.60;
    private final double FAR_SHOT_HOOD = 0.45;

    // physical limits of the hood servo
    private final double MAX_HIGH_HOOD_POSITION = 0.85; // TODO
    private final double MAX_LOW_HOOD_POSITION = 0.6; // TODO

    private double hoodPosition = MAX_LOW_HOOD_POSITION;

    private boolean outtakeOn = false; // start the match with outtake on
    private boolean lastAState = false;

    // PIDF tuning resources: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    // After kV is set, tune kP to minimize error, use small increases
    @Config
    static class PIDFCoefficients {
        public static double kP = 24;
        public static double kI = 0;
        public static double kD = 0.08;
        public static double kF = 15;
        public static double targetVelocity = 1000.0;
    }

    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "rightShooter");

        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setVelocityPIDFCoefficients(PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kF);
        outtakeMotor2.setVelocityPIDFCoefficients(PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kF);
        hoodServo.setPosition(hoodPosition);
    }

    public void testOuttake(boolean a, Telemetry telemetry, Gamepad gamepad) {
        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }

        if (gamepad.dpad_up) {
            PIDFCoefficients.targetVelocity += 10;
        } else if (gamepad.dpad_down) {
            PIDFCoefficients.targetVelocity -= 10;
        }

        lastAState = a;

        // Apply velocity control
        if (outtakeOn) {
            outtakeMotor.setVelocity(PIDFCoefficients.targetVelocity);
            outtakeMotor2.setVelocity(-PIDFCoefficients.targetVelocity);
        } else {
            outtakeMotor.setVelocity(0);
            outtakeMotor2.setVelocity(0);
        }

        if (gamepad.dpad_left) {
            hoodPosition = Math.min(hoodPosition + 0.04, 1.0);
        }
        else if (gamepad.dpad_right) {
            hoodPosition = Math.max(hoodPosition - 0.04, 0.0);
        }
        hoodServo.setPosition(hoodPosition);

        // Telemetry
        double ticksPerSecond = outtakeMotor.getVelocity();
        double ticksPerSecond2 = outtakeMotor2.getVelocity();

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", PIDFCoefficients.targetVelocity);
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Current Velocity Left Shooter (ticks/sec)", ticksPerSecond);
        telemetry.addData("Current Velocity Right Shooter (ticks/sec)", ticksPerSecond2);
    }

    public void runCameraShots(Gamepad gamepad, Telemetry telemetry, double xDist) {
        boolean a = gamepad.a;

        // update target velocity based on distance to goal if needed
        // Toggle motor on/off
        if (a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = a;

        // Apply velocity control
        if (outtakeOn) {
            outtakeMotor.setVelocity(PIDFCoefficients.targetVelocity);
            outtakeMotor2.setVelocity(-PIDFCoefficients.targetVelocity);
        } else {
            outtakeMotor.setVelocity(0);
            outtakeMotor2.setVelocity(0);
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
        // NEW
        targetVelocityTicks =
                -0.000706398 * x * x * x
                        + 0.218664 * x * x
                        - 27.87962 * x
                        + 25.73488;
//        targetVelocityTicks = -6.86887 * x -590.15262;
        targetVelocityTicks = Range.clip(targetVelocityTicks, -1530, -1000);
    }

    public void dynamicallyUpdateHoodPositionByOdometry(double x) {
        // the hood will be a function of the shooter velocity
        // to allow for a velocity-based hood for rapid firing

        double v = outtakeMotor.getVelocity();
        if (x < 120) {
            // close shot cubic regression
            hoodPosition =
                    -(9.83951e-9) * v * v * v
                            - 0.0000367909 * v * v
                            - 0.0444431 * v
                            - 16.73497;
        }
        // have a separate regression to handle for far shot because not modeled well experimentally
        else {
            // far shot quadratic regression
            // y = -0.00000117647x^{2}-0.0011x+1.47147
            hoodPosition =
                    -0.00000117647 * v * v
                            - 0.0011 * v
                            + 1.47147;
        }

        // keep hood in the down position
        if (Math.abs(v) < 900) {
            hoodPosition = 0.75;
        }

        hoodPosition = Range.clip(hoodPosition, 0.35, 0.75);
        hoodServo.setPosition(hoodPosition);
    }

    public boolean isAtShootingSpeed() {
        double leftShooterVelocity = outtakeMotor.getVelocity();
        double rightShooterVelocity = outtakeMotor.getVelocity(); // is negative ticks

        double averageVelocity = (Math.abs(leftShooterVelocity) + Math.abs(rightShooterVelocity)) / 2.0;

        return Math.abs(averageVelocity - targetVelocityTicks) < ACCEPTABLE_VELOCITY_ERROR_TICKS;
    }

    public void applyVelocity(boolean onState, double targetVelocity) {
        if (onState) {
            outtakeMotor.setVelocity(targetVelocity);
            outtakeMotor2.setVelocity(-targetVelocity);
        } else {
            outtakeMotor.setVelocity(0);
            outtakeMotor2.setVelocity(0);
        }
    }
}
