package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HoodOuttakeTest extends OpMode {
    public Servo hoodServo;
    public DcMotorEx outtakeMotor; //output velocity in ticks and angles into telemetry

    public double leftPosition = 0.0;
    public double rightPosition = 1.0;

    public double velocity;

    public double motorPower = 1.0;

    private boolean outtakeOn = false;
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    static final double TICKS_PER_REV = 537.7;


    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(leftPosition); // Set to a neutral position
    }

    @Override
    public void loop() {
        // Toggle outtake motor on/off with gamepad1.a
        if (gamepad1.a && !lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = gamepad1.a;

        if (outtakeOn) {
            outtakeMotor.setPower(motorPower);
        } else {
            outtakeMotor.setPower(0.0);
        }

        // Dpad left/right to adjust hoodServo position
        if (gamepad1.dpad_left && !lastDpadLeft) {
            leftPosition -= 0.05;
            if (leftPosition < 0.0) leftPosition = 0.0;
            hoodServo.setPosition(leftPosition);
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) {
            leftPosition += 0.05;
            if (leftPosition > 1.0) leftPosition = 1.0;
            hoodServo.setPosition(leftPosition);
        }
        lastDpadRight = gamepad1.dpad_right;

        // Dpad up/down to adjust motorPower
        if (gamepad1.dpad_up && !lastDpadUp) {
            motorPower += 0.05;
            if (motorPower > 1.0) motorPower = 1.0;
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            motorPower -= 0.05;
            if (motorPower < 0.0) motorPower = 0.0;
        }
        lastDpadDown = gamepad1.dpad_down;

        velocity = outtakeMotor.getVelocity();
        double ticksPerSecond = outtakeMotor.getVelocity(); // already in ticks/sec
        double revPerSecond = ticksPerSecond / TICKS_PER_REV;
        double radPerSecond = revPerSecond * 2 * Math.PI;

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Servo Position", leftPosition);
        telemetry.addData("Velocity (ticks/sec)", ticksPerSecond);
        telemetry.addData("Velocity (rev/sec)", revPerSecond);
        telemetry.addData("Angular Velocity (rad/sec)", radPerSecond);
        telemetry.update();
    }
}