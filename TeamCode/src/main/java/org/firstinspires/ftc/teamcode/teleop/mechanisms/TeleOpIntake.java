package org.firstinspires.ftc.teamcode.teleop.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpIntake {
    public DcMotor intakeMotor;

    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }
}
