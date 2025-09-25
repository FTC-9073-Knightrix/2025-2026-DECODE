package org.firstinspires.ftc.teamcode.teleop.main;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    boolean rightStickPrevPressed = false;

    @Override
    public void init() {super.init();}
    // put robot methods you will run in the teleOp here
    public void runToggledDrive() {
        boolean rightStickPressed = gamepad1.right_stick_button;
        if (rightStickPressed && !rightStickPrevPressed) {
            if (drive.getDriveMode() == TeleOpMecanumDrive.DriveMode.MANUAL) {
                drive.setDriveMode(TeleOpMecanumDrive.DriveMode.LOCKED_ON);
            } else {
                drive.setDriveMode(TeleOpMecanumDrive.DriveMode.MANUAL);
            }
            rightStickPrevPressed = true;
        }

        if (!rightStickPressed && rightStickPrevPressed) {
            rightStickPrevPressed = false;
        }
    }
    public void displayTelemetry() {
        telemetry.addData("Runtime: ", getRuntime());
    }
}
