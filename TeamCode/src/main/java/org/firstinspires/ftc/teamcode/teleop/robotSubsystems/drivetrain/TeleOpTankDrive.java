package org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpTankDrive {
    public DcMotor leftDrive;
    public DcMotor rightDrive;

    public void init(HardwareMap hwMap) {
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
    }

    public void runTankDrive(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

}
