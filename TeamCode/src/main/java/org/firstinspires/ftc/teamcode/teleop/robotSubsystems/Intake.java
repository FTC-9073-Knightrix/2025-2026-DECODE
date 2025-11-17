package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intakeMotor;
    private boolean isTogglePressed;
    private boolean intakeOn;

    private final double INTAKE_IN_POWER = 1.0;
    private final double INTAKE_OUT_POWER = -1.0;
    private final double INTAKE_STOP_POWER = 0.0;
    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intake");
        isTogglePressed = false;
        intakeOn = false;
    }

    public void runIntake(boolean toggleButton) {
        if (toggleButton && !isTogglePressed) {
            intakeOn = !intakeOn;
        }
        isTogglePressed = toggleButton;

        if (intakeOn) {
            intakeMotor.setPower(INTAKE_IN_POWER);
        }
        else {
            intakeMotor.setPower(INTAKE_STOP_POWER);
        }
    }

    public void ejectIntake() {
        intakeMotor.setPower(INTAKE_OUT_POWER);
    }
}
