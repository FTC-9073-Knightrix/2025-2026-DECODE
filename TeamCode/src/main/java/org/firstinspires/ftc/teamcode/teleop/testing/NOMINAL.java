package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.left_trigger > 0.5) {
            shooter.setVelocity(2000);
        } else {
            shooter.setPower(0);
        }
        shooter.setVelocityPIDFCoefficients(RB.kP, RB.kI, RB.kD, RB.kF);

        dashboardTelemetry.addData("kP", RB.kP);
        dashboardTelemetry.addData("kI", RB.kI);
        dashboardTelemetry.addData("kD", RB.kD);
        dashboardTelemetry.addData("kF", RB.kF);
        dashboardTelemetry.update();

        telemetry.addData("Shooter Voltage", voltageSensor.getVoltage());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Target Velocity", "2000");
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("PIDF Coefficients", shooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).toString());
        telemetry.update();
    }
}
