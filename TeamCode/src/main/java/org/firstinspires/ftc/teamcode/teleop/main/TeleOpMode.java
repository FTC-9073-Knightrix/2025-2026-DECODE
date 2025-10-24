package org.firstinspires.ftc.teamcode.teleop.main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "1: DECODE TeleOp")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;
    private boolean outtakeOn = false;
    private boolean lastAState = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    @Override
    public void loop() {
        // THIS IS THE MAIN RUN LOOP FOR THE ROBOT
        // PUT METHODS IN HERE THAT U WANT TO BE CONTINUOUSLY RUNNING
        // WHILE WE R CONTROLLING THE ROBOT
        runToggledDrive();
        runVision();
        displayTelemetry(); // this shows text on the driver hub
        // Toggle motor on/off
        if (gamepad1.a && ! lastAState) {
            outtakeOn = !outtakeOn;
        }
        lastAState = gamepad1.a;

        // Servo adjust
        if (gamepad1.dpad_left && !lastDpadLeft) {
            hoodPosition = Math.min(1.0, hoodPosition + 0.05);
            hoodServo.setPosition(hoodPosition);
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) {
            hoodPosition = Math.max(0.0, hoodPosition - 0.05);
            hoodServo.setPosition(hoodPosition);
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
//        if (outtakeOn) {
//            outtakeMotor.setVelocity(targetVelocityTicks);
//        } else {
//            outtakeMotor.setVelocity(0);
//        }

        // Telemetry
//        double ticksPerSecond = outtakeMotor.getVelocity();
//        double revPerSecond = ticksPerSecond / TICKS_PER_REV;
//        double radPerSecond = revPerSecond * 2 * Math.PI;

        telemetry.addData("Outtake On", outtakeOn);
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocityTicks);
//        telemetry.addData("Current Velocity (ticks/sec)", ticksPerSecond);
//        telemetry.addData("Velocity (deg/sec)", outtakeMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("Velocity (rev/sec)", revPerSecond);
//        telemetry.addData("Angular Velocity (rad/sec)", radPerSecond);
        telemetry.addData("Servo Position", hoodPosition);
        telemetry.update();
    }
}