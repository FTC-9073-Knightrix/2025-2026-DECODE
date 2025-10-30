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
    VoltageSensor batteryVoltageSensor;

    private double hoodPosition = 0.0;
    static final double TICKS_PER_REV = 28; // FOR GO BILDA 5202 SERIES
    public double targetVelocityTicks = 0.0;

    // TODO CHANGE THESE TEMP VALUES BASED ON TESTING
    private final double FAR_SHOT_VELOCITY_TICKS = 1500.0;
    private final double MID_SHOT_VELOCITY_TICKS = 1250.0;
    private final double NEAR_SHOT_VELOCITY_TICKS = 1000.0;

    private boolean outtakeOn = false;
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // PIDF tuning resources: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    // TODO TUNE kV
    private final double kV = 1.0 / 3120.0; // TODO: MEASURE THIS (V_MIN / V_CURR) / (ticks per second at power (V_MIN, / V_CURR))
    private final double NOMINAL_VOLTAGE = 10.0; // The voltage the kV was measured at

    // TODO TUNE kP
    // After kV is set, tune kP to minimize error, use small increases
    private final double kP = 0.00;


    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.get("Control Hub");

        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        double kF = kV * NOMINAL_VOLTAGE / currentBatteryVoltage;
        outtakeMotor.setVelocityPIDFCoefficients(kP, 0, 0, kF); // we use only kP and kF here
        hoodServo.setPosition(hoodPosition);
    }

    public void runOuttake(boolean a, boolean dpad_left, boolean dpad_right,
                           boolean dpad_up, boolean dpad_down,
                           Telemetry telemetry) {

        // on every loop cycle, update the feedforward coefficient to account for changing battery voltage
        // this makes it so a drop in battery voltage doesn't cause a drop in flywheel velocity
        updateFeedForwardCoefficient();
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
            targetVelocityTicks += 50;
            if (targetVelocityTicks > 2000) targetVelocityTicks = 2000;
        }
        lastDpadUp = dpad_up;

        if (dpad_down && !lastDpadDown) {
            targetVelocityTicks -= 100;
            if (targetVelocityTicks < 0) targetVelocityTicks = 0;
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
        double revPerSecond = ticksPerSecond / TICKS_PER_REV;
        double revPerMinute = revPerSecond * 60;
        double radPerSecond = revPerSecond * 2 * Math.PI;

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Velocity (deg/sec)", outtakeMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Velocity (rev/sec)", revPerSecond);
        telemetry.addData( "Velocity (rev/min or RPM)", revPerMinute);
        telemetry.addData("Angular Velocity (rad/sec)", radPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
        telemetry.update();
    }

    private void updateFeedForwardCoefficient() {
        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        double kF = kV * NOMINAL_VOLTAGE / currentBatteryVoltage;
        outtakeMotor.setVelocityPIDFCoefficients(0.01, 0, 0, kF); // we use only kP and kF here
    }

    public void dynamicallyUpdateHoodPosition(double horizontalDistanceToGoalInches) {
        if (horizontalDistanceToGoalInches < 0) {
            // No valid distance, do not update hood position
            return;
        }

        double x = horizontalDistanceToGoalInches;
        // TODO Create a regression here based on empirical data points (desmos.com)
        double targetHoodPosition = 0.0000025 * x * x - 0.0015 * x + 0.85; // a filler regression curve
        double deltaPos = targetHoodPosition - hoodPosition;

        if (Math.abs(deltaPos) < 0.05) {
            hoodPosition = targetHoodPosition;
        } else {
            double adjustment = deltaPos > 0 ? 0.05 : -0.05;
            hoodPosition += adjustment;
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

        if (distance > 100) { // FAR SHOT
            targetVelocityTicks = FAR_SHOT_VELOCITY_TICKS;
        } else if (distance > 30) { // MID SHOT
            targetVelocityTicks = MID_SHOT_VELOCITY_TICKS;
        } else { // NEAR SHOT
            targetVelocityTicks = NEAR_SHOT_VELOCITY_TICKS;
        }
    }
}
