package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Config
class TURRET_PID {
    // in a class with @Config

    public static PIDCoefficients coefficients = new PIDCoefficients(0.02, 0, 0);

    // Use a position PID so KineticState uses position units (degrees) for goal & state
    static ControlSystem turretControlSystem = ControlSystem.builder()
            .posPid(coefficients)
            .build();
}

public class Turret {
    public DcMotorEx turretMotor;
    private TouchSensor turretSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    // EMPIRICAL MEASUREMENT: 700 ticks per 180 degrees
    private static final double TICKS_PER_180_DEGREES = 700.0;

    private double convertDegreesToTicks(double angleDegrees) {
        // Empirical conversion: 700 ticks = 180 degrees
        return (angleDegrees / 180.0) * TICKS_PER_180_DEGREES;
    }

    private double convertTickstoDegrees(double ticks) {
        // Empirical conversion: 700 ticks = 180 degrees
        return (ticks / TICKS_PER_180_DEGREES) * 180.0;
    }

    private double targetAngle = 0.0;
    private double targetTicks = 0.0;

    private double currentAngle = 0.0;
    public double angleError = 0.0;

    double turnPower = 0.0;
    public void init(HardwareMap hw) {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        turretMotor = hw.get(DcMotorEx.class, "turret");
        turretSensor = hw.get(TouchSensor.class, "turretSensor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_USING_ENCODER so velocity/encoder-backed features are available to ControlSystem
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        currentAngle = 0.0;
        targetAngle = 0.0;
        targetTicks = 0.0;
        turretMotor.setPower(0.0);
        turretTimer.reset();
    }

    public void run(double offsetDegrees, Gamepad gamepad, Telemetry telemetry) {

        // update current pose/encoders
        currentAngle = convertTickstoDegrees(turretMotor.getCurrentPosition());

        // If requested target is outside safe travel range, do NOT attempt to aim.
        // This prevents commanding the turret into mechanical end-stops or dangerous rotations.
        final double MAX_SAFE_ANGLE = 165.0; // degrees on either side
        if (Math.abs(offsetDegrees) > MAX_SAFE_ANGLE) {
            // Keep the turret stopped and report the out-of-bounds target for diagnostics
            turnPower = 0.0;
            turretMotor.setPower(0.0);

            // Update angle error for diagnostics but do NOT change the control goal
            updateAngleError(offsetDegrees);

            return;
        }

        // probably do not use this -- the magnet switch
//        relocalizeTurret();

        // Set target state so diagnostics/telemetry match
        setTarget(offsetDegrees);
        updateAngleError(offsetDegrees);

        // Use angles (degrees) as the units for the ControlSystem goal and current state
        TURRET_PID.turretControlSystem.setGoal(new KineticState(targetAngle));

        // include angular velocity (deg/s) converted from ticks/sec
        double currentAngularVelocityDegPerSec = convertTickstoDegrees(turretMotor.getVelocity());
        double rawOutput = TURRET_PID.turretControlSystem.calculate(new KineticState(currentAngle, currentAngularVelocityDegPerSec));

        double maxPower = 0.65;
        turnPower = Range.clip(rawOutput, -maxPower, maxPower);

        // Enforce software travel limits (this modifies turnPower to prevent driving into hard stops)
        keepTurretWithinLimits();

        // If the angular error is greater than 0.5 degrees, ensure at least 0.05 magnitude power is applied
        double minPowerForAlignment = 0.05;
        double alignmentThreshold = 0.5; // degrees
        if (Math.abs(angleError) > alignmentThreshold) {
            if (Math.abs(turnPower) < minPowerForAlignment) {
                // If controller produced near-zero output, use the sign of the angle error to pick direction
                if (Math.abs(rawOutput) > 1e-6) {
                    turnPower = Math.signum(rawOutput) * minPowerForAlignment;
                } else {
                    turnPower = Math.signum(angleError) * minPowerForAlignment;
                }
            }
            turnPower = Range.clip(turnPower, -maxPower, maxPower);
        }

        // Apply power to the motor
        turretMotor.setPower(turnPower);

        // telemetry
        telemetry.addData("Target Turret angle", targetAngle);
        telemetry.addData("Target turret ticks", targetTicks);
        telemetry.addData("Current Turret angle", currentAngle);
        telemetry.addData("turret ticks", turretMotor.getCurrentPosition());
        telemetry.addData("turn power", turnPower);

        dashboardTelemetry.addData("target turret angle", targetAngle);
        dashboardTelemetry.addData("current turret angle", currentAngle);
    }

    private void setTarget(double angleDegrees) {
        targetAngle = angleDegrees;
        targetTicks = convertDegreesToTicks(targetAngle);
    }

    private double lastAngleError = 0.0;
    private double integralSum = 0.0;
    ElapsedTime turretTimer = new ElapsedTime();
    public void alignTurret(double offsetDegrees) {
        setTarget(offsetDegrees);

        double maxPower = 0.6; // maximum turn power
        double alignmentThreshold = 0.5; // degrees - reduced for tighter alignment
        double minPower = 0.08; // minimum power to overcome static friction

        double angleError = offsetDegrees - currentAngle;

        if (Math.abs(angleError) > alignmentThreshold) {
            double dt = Math.max(turretTimer.seconds(), 0.001); // guard against zero time interval

            // Accumulate error for integral term
            integralSum += angleError * dt;

            // Prevent integral windup
            double maxIntegral = 10.0;
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

            double derivative = (angleError - lastAngleError) / dt;
            derivative = Range.clip(derivative, -100.0, 100.0);

            // D term should OPPOSE the rate of change to provide damping
            turnPower = (TURRET_PID.coefficients.kP * angleError) + (TURRET_PID.coefficients.kI * integralSum) - (TURRET_PID.coefficients.kD * derivative);

            // Add minimum power to overcome static friction when close to target
            if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            turnPower = Range.clip(turnPower, -maxPower, maxPower);
        } else {
            // Reset integral when aligned
            integralSum = 0.0;
            turnPower = 0.0;
        }

        lastAngleError = angleError;
        turretTimer.reset(); // Reset AFTER reading dt

    }

    private void keepTurretWithinLimits() {
        double curTicks = turretMotor.getCurrentPosition();
        double oneSideBound = 440;

        // po
        if (curTicks < -oneSideBound) {
            turnPower = Math.max(0, turnPower);
        }
        else if (curTicks > oneSideBound) {
            turnPower = Math.min(0, turnPower);
        }
    }

    // DO NOT USE
    private void relocalizeTurret() {
        if (turretSensor.isPressed()) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentAngle = 0.0;
        }
    }

    public boolean isAligned() {
        double angleError = getAngleError();
        double alignmentThreshold = 1.0; // degrees
        return Math.abs(angleError) <= alignmentThreshold;
    }

    private double getAngleError() {
        return angleError;
    }

    private void updateAngleError(double offsetAngle) {
        angleError = offsetAngle - currentAngle;
    }

    public void stop() {
        turnPower = 0;
        lastAngleError = 0;
        integralSum = 0;
    }
}
