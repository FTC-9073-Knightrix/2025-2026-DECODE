package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class HoodOuttakeTest extends OpMode {
    public Servo hoodServo;
    public DcMotorEx outtakeMotor;

    public double leftPosition = 0.0;
    public double rightPosition = 1.0;

    private boolean outtakeOn = false;
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    static final double TICKS_PER_REV = 28;
    public double targetVelocityTicks = 0.0;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(leftPosition);
    }

    @Override
    public void loop() {
        // Toggle motor on/off
        if (gamepad1.a && !lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = gamepad1.a;

        // Servo adjust
        if (gamepad1.dpad_left && !lastDpadLeft) {
            leftPosition = Math.max(0.0, leftPosition - 0.05);
            hoodServo.setPosition(leftPosition);
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) {
            leftPosition = Math.min(1.0, leftPosition + 0.05);
            hoodServo.setPosition(leftPosition);
        }
        lastDpadRight = gamepad1.dpad_right;

        // Adjust velocity
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetVelocityTicks += 100;
            if (targetVelocityTicks > 2000) targetVelocityTicks = 2000;
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            targetVelocityTicks -= 100;
            if (targetVelocityTicks < 0) targetVelocityTicks = 0;
        }
        lastDpadDown = gamepad1.dpad_down;

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
        telemetry.addData("Servo Position", leftPosition);
        telemetry.update();
    }
}