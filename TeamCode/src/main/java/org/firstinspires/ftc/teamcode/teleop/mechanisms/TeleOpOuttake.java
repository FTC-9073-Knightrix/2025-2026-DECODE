package org.firstinspires.ftc.teamcode.teleop.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TeleOpOuttake {
    public Servo hoodServo;
    public DcMotorEx outtakeMotor;

    public double hoodPosition = 0.0;
    static final double TICKS_PER_REV = 28;
    public double targetVelocityTicks = 0.0;

    private boolean outtakeOn = false;
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(hoodPosition);
    }

    public void runOuttake(boolean a, boolean dpad_left, boolean dpad_right,
                           boolean dpad_up, boolean dpad_down,
                           Telemetry telemetry) {
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
            targetVelocityTicks += 100;
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
        double radPerSecond = revPerSecond * 2 * Math.PI;

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Velocity (deg/sec)", outtakeMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Velocity (rev/sec)", revPerSecond);
        telemetry.addData("Angular Velocity (rad/sec)", radPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
        telemetry.update();
    }
}
