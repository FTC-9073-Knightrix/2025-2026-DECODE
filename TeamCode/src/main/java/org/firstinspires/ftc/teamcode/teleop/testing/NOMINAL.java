package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
class RB {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
}

@TeleOp
public class NOMINAL extends OpMode {
    DcMotorEx shooter;
    VoltageSensor voltageSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    double NOMINAL_VOLTAGE = 10.0;

    // Fields for tuning via gamepad
    double targetVelocity = 1250.0;
    int selectedCoeff = 0; // 0 = kP, 1 = kI, 2 = kD, 3 = kF
    final double COEFF_STEP = 0.01;
    final long COEFF_ADJUST_INTERVAL_MS = 150;
    long lastCoeffAdjustMs = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.left_trigger > 0.5) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }

        // Select which coefficient: A -> kP, X -> kI, Y -> kD, B -> kF
        if (gamepad1.a) selectedCoeff = 0;
        if (gamepad1.x) selectedCoeff = 1;
        if (gamepad1.y) selectedCoeff = 2;
        if (gamepad1.b) selectedCoeff = 3;

        // Adjust selected coefficient with left stick (throttled)
        double stickY = gamepad1.left_stick_y;
        long now = System.currentTimeMillis();
        if ((stickY < -0.5 || stickY > 0.5) && (now - lastCoeffAdjustMs > COEFF_ADJUST_INTERVAL_MS)) {
            double change = (stickY < -0.5) ? COEFF_STEP : -COEFF_STEP;
            switch (selectedCoeff) {
                case 0:
                    RB.kP += change * 20;
                    if (RB.kP < 0) RB.kP = 0;
                    break;
                case 1:
                    RB.kI += change;
                    if (RB.kI < 0) RB.kI = 0;
                    break;
                case 2:
                    RB.kD += change;
                    if (RB.kD < 0) RB.kD = 0;
                    break;
                case 3:
                    RB.kF += change;
                    if (RB.kF < 0) RB.kF = 0;
                    break;
            }
            lastCoeffAdjustMs = now;
        }

        shooter.setVelocityPIDFCoefficients(RB.kP, RB.kI, RB.kD, RB.kF);

        dashboardTelemetry.addData("kP", RB.kP);
        dashboardTelemetry.addData("kI", RB.kI);
        dashboardTelemetry.addData("kD", RB.kD);
        dashboardTelemetry.addData("kF", RB.kF);
        dashboardTelemetry.addData("Target Velocity", targetVelocity);
        dashboardTelemetry.addData("Shooter Velocity", shooter.getVelocity());

        String selectedName;
        switch (selectedCoeff) {
            case 0: selectedName = "kP"; break;
            case 1: selectedName = "kI"; break;
            case 2: selectedName = "kD"; break;
            case 3: selectedName = "kF"; break;
            default: selectedName = "?"; break;
        }
        dashboardTelemetry.addData("Selected", selectedName);
        dashboardTelemetry.update();

        telemetry.addData("Shooter Voltage", voltage);
        telemetry.addData("Nominal Voltage", NOMINAL_VOLTAGE);
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("PIDF Coefficients", shooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("Selected Coeff", selectedName);
        telemetry.update();
    }
}
