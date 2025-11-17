package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Flywheel Tuning", group = "Testing")
public class FlyWheelTuning extends OpMode {

    private DcMotorEx outtakeMotor;
    private VoltageSensor batteryVoltageSensor;

    private static final double TICKS_PER_REV = 28.0; // GO BILDA 5202 SERIES
    private static final double NOMINAL_VOLTAGE = 10.0; // Voltage at which you measured kV

    // Tunable parameters
    private double kV = 0.85 / 1880.0; // Start with your initial guess
    private double kP = 10.0;
    private double targetVelocityTicks = 1000.0;

    // Button state tracking
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // Mode tracking
    private boolean motorRunning = false;
    private TuningMode currentMode = TuningMode.TARGET_VELOCITY;

    private enum TuningMode {
        TARGET_VELOCITY,
        KV_TUNING,
        KP_TUNING
    }

    @Override
    public void init() {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        batteryVoltageSensor = hardwareMap.voltageSensor.get("Control Hub");

        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        updatePIDFCoefficients(kV, kP);

        telemetry.addLine("Flywheel Tuning OpMode");
        telemetry.addLine("Press START to begin");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Toggle motor on/off");
        telemetry.addLine("X: Switch to Target Velocity mode");
        telemetry.addLine("Y: Switch to kV tuning mode");
        telemetry.addLine("B: Switch to kP tuning mode");
        telemetry.addLine("DPad Up/Down: Adjust current parameter (large)");
        telemetry.addLine("DPad Left/Right: Adjust current parameter (small)");
        telemetry.addLine("Left/Right Bumper: Quick velocity presets");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleInput();
        updateMotor();
        displayTelemetry();
    }

    private void handleInput() {
        // Toggle motor on/off
        if (gamepad1.a && !lastA) {
            motorRunning = !motorRunning;
            if (!motorRunning) {
                outtakeMotor.setVelocity(0);
            }
        }
        lastA = gamepad1.a;

        // Switch tuning modes
        if (gamepad1.x && !lastX) {
            currentMode = TuningMode.TARGET_VELOCITY;
        }
        lastX = gamepad1.x;

        if (gamepad1.y && !lastY) {
            currentMode = TuningMode.KV_TUNING;
        }
        lastY = gamepad1.y;

        if (gamepad1.b && !lastB) {
            currentMode = TuningMode.KP_TUNING;
        }
        lastB = gamepad1.b;

        // Adjust parameters based on current mode
        switch (currentMode) {
            case TARGET_VELOCITY:
                if (gamepad1.dpad_up && !lastDpadUp) {
                    targetVelocityTicks += 100;
                    targetVelocityTicks = Math.min(targetVelocityTicks, 3500);
                }
                if (gamepad1.dpad_down && !lastDpadDown) {
                    targetVelocityTicks -= 100;
                    targetVelocityTicks = Math.max(targetVelocityTicks, 0);
                }
                if (gamepad1.dpad_right && !lastDpadRight) {
                    targetVelocityTicks += 10;
                    targetVelocityTicks = Math.min(targetVelocityTicks, 3500);
                }
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    targetVelocityTicks -= 10;
                    targetVelocityTicks = Math.max(targetVelocityTicks, 0);
                }
                break;

            case KV_TUNING:
                if (gamepad1.dpad_up && !lastDpadUp) {
                    kV += 0.001;
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_down && !lastDpadDown) {
                    kV -= 0.001;
                    kV = Math.max(kV, 0.0);
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_right && !lastDpadRight) {
                    kV += 0.0001;
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    kV -= 0.0001;
                    kV = Math.max(kV, 0.0);
                    updatePIDFCoefficients(kV, kP);
                }
                break;

            case KP_TUNING:
                if (gamepad1.dpad_up && !lastDpadUp) {
                    kP += 0.1;
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_down && !lastDpadDown) {
                    kP -= 0.1;
                    kP = Math.max(kP, 0.0);
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_right && !lastDpadRight) {
                    kP += 0.01;
                    updatePIDFCoefficients(kV, kP);
                }
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    kP -= 0.01;
                    kP = Math.max(kP, 0.0);
                    updatePIDFCoefficients(kV, kP);
                }
                break;
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Quick velocity presets
        if (gamepad1.left_bumper && !lastLeftBumper) {
            targetVelocityTicks = 1000.0; // Low speed
        }
        lastLeftBumper = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !lastRightBumper) {
            targetVelocityTicks = 2000.0; // High speed
        }
        lastRightBumper = gamepad1.right_bumper;
    }

    private void updateMotor() {
        if (motorRunning) {
            outtakeMotor.setVelocity(targetVelocityTicks);
        }
    }

    private void updatePIDFCoefficients(double kF, double kP) {
        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        outtakeMotor.setVelocityPIDFCoefficients(kP, 0, 0, kF);
    }

    private void displayTelemetry() {
        double currentVelocityTicks = outtakeMotor.getVelocity();
        double error = targetVelocityTicks - currentVelocityTicks;
        double percentError = targetVelocityTicks > 0 ? (error / targetVelocityTicks) * 100 : 0;

        double revPerSecond = currentVelocityTicks / TICKS_PER_REV;
        double rpm = revPerSecond * 60;

        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        double kF = kV * NOMINAL_VOLTAGE / currentBatteryVoltage;

        telemetry.addLine("=== FLYWHEEL TUNING ===");
        telemetry.addLine();

        telemetry.addData("Motor Running", motorRunning ? "YES" : "NO");
        telemetry.addData("Current Mode", currentMode.toString());
        telemetry.addLine();

        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Target (ticks/sec)", "%.1f", targetVelocityTicks);
        telemetry.addData("Current (ticks/sec)", "%.1f", currentVelocityTicks);
        telemetry.addData("Error (ticks/sec)", "%.1f", error);
        telemetry.addData("Error (%)", "%.2f%%", percentError);
        telemetry.addData("RPM", "%.1f", rpm);
        telemetry.addLine();

        telemetry.addLine("--- TUNING PARAMETERS ---");
        telemetry.addData("PIDF Coefficients", outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addLine();

        telemetry.addLine("--- SYSTEM INFO ---");
        telemetry.addData("Battery Voltage", "%.2f V", currentBatteryVoltage);
        telemetry.addData("Nominal Voltage", "%.1f V", NOMINAL_VOLTAGE);
        telemetry.addLine();

        telemetry.addLine("--- TUNING GUIDE ---");
        telemetry.addLine("STEP 1 - Measure kV:");
        telemetry.addLine("  1. Set kP to 0");
        telemetry.addLine("  2. Run motor at target speed");
        telemetry.addLine("  3. Note steady-state velocity");
        telemetry.addLine("  4. kV = NOMINAL_VOLTAGE / steady_state_velocity");
        telemetry.addLine();
        telemetry.addLine("STEP 2 - Tune kP:");
        telemetry.addLine("  1. With kV set, start with kP = 0");
        telemetry.addLine("  2. Gradually increase kP");
        telemetry.addLine("  3. Stop when error is minimized");
        telemetry.addLine("  4. If oscillating, reduce kP");
        telemetry.addLine();

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A", "Toggle Motor");
        telemetry.addData("X", "Velocity Mode");
        telemetry.addData("Y", "kV Mode");
        telemetry.addData("B", "kP Mode");
        telemetry.addData("DPad Up/Down", "Large adjust");
        telemetry.addData("DPad L/R", "Small adjust");
        telemetry.addData("Bumpers", "Velocity presets");

        telemetry.update();
    }

    @Override
    public void stop() {
        outtakeMotor.setVelocity(0);
    }
}
