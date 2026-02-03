// java
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
    private final double MAX_HIGH_HOOD_POSITION = 0; // TODO
    private final double MAX_LOW_HOOD_POSITION = 1.0; // TODO

    private double hoodPosition = MAX_LOW_HOOD_POSITION;

    private boolean outtakeOn = false; // start the match with outtake on
    private boolean lastAState = false;

    // PIDF tuning resources: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    // After kV is set, tune kP to minimize error, use small increases
    @Config
    static class PIDFCoefficients {
        public static double kP = 23.8;
        public static double kI = 0;
        public static double kD = 0.08;
        public static double kF = 14;
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
        applyVelocity(outtakeOn, PIDFCoefficients.targetVelocity);

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

        // update target velocity based on distance to goal if needed
        updateCameraShotSpeed(xDist);
        updateHoodByVelocity();
        // Toggle motor on/off

        if (gamepad.a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = gamepad.a;
        applyVelocity(outtakeOn, targetVelocityTicks);

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Current Velocity (ticks/sec)", getAverageVelocity());
    }

    public void updateCameraShotSpeed(double x) {
        if (x < 0) {
            // No valid distance, do not update velocity
            return;
        }

        // ~1050 for close, ~1150-1250 for mid distances, ~1500 for far distances.
        double regressionVelocity =
                -0.00025 * x * x * x
                        + 0.037 * x * x
                        + 3.8 * x
                        + 880.0;

        targetVelocityTicks = Range.clip(regressionVelocity, NEAR_SHOT_VELOCITY_TICKS, 1520);

    }
    // A VELOCITY BASED HOOD
    private void updateHoodByVelocity() {
        double v = getAverageVelocity();

        // regression from desmos of hood position plotted vs velocity
        double y = -0.00148148 * v + 2.56759;
        hoodPosition = Range.clip(y, 0.3, 1.0);

        hoodServo.setPosition(hoodPosition);
    }

    public void runOdometryShots(Gamepad gamepad, Telemetry telemetry, double xDist) {
        // Update the target velocity first (so switching modes updates the setpoint)
        updateOdometryShotSpeed(xDist);
        updateHoodByVelocity();

        // Toggle motor on/off (same debounce logic as camera mode)
        if (gamepad.a && !lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = gamepad.a;

        // Apply the (possibly updated) velocity to the motors
        applyVelocity(outtakeOn, targetVelocityTicks);

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Current Velocity (ticks/sec)", getAverageVelocity());
    }

    private void updateOdometryShotSpeed(double x) {
        if (x < 0) {
            // No valid distance
            return;
        }

        double regressionVelocity = 6.37682 * x + 629.08496;

        targetVelocityTicks = Range.clip(
                regressionVelocity,
                1000,
                1400
        );
    }

    public boolean isAtShootingSpeed() {
        double averageVelocity = getAverageVelocity();
        return Math.abs(averageVelocity - targetVelocityTicks) < ACCEPTABLE_VELOCITY_ERROR_TICKS;
    }

    private double getAverageVelocity() {
        double leftShooterVelocity = outtakeMotor.getVelocity();
        double rightShooterVelocity = outtakeMotor2.getVelocity(); // is negative ticks

        return (Math.abs(leftShooterVelocity) + Math.abs(rightShooterVelocity)) / 2.0;
    }

    private void applyVelocity(boolean onState, double targetVelocity) {
        if (onState) {
            outtakeMotor.setVelocity(targetVelocity);
            outtakeMotor2.setVelocity(-targetVelocity); // the right shooter needs to be negative
        } else {
            outtakeMotor.setVelocity(0);
            outtakeMotor2.setVelocity(0);
        }
    }
}