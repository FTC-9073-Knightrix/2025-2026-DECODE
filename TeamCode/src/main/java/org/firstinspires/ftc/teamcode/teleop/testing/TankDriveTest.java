package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpTankDrive;

@TeleOp(name = "Tank Drive TeleOp")
public class TankDriveTest extends OpMode {
    TeleOpTankDrive drive = new TeleOpTankDrive();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        // keep power low for outreach events
        double leftPower = -gamepad1.left_stick_y * 0.3;
        double rightPower = -gamepad1.right_stick_y * 0.3;

        drive.runTankDrive(leftPower, rightPower);
    }

}
