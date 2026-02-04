package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx intakeMotor;
    public DcMotor intakeMotor2;
    private boolean isTogglePressed;
    private boolean intakeOn;

    private final double INTAKE_IN_POWER = -1.0;
    private final double INTAKE_OUT_POWER = 1.0;
    private final double INTAKE_STOP_POWER = 0.0;

    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "intake");
        intakeMotor2 = hw.get(DcMotor.class, "intake2");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isTogglePressed = false;
        intakeOn = false; // start the match with intake on
    }

    public void runIntake(boolean toggleButton, boolean ejectButton) {
        // Force Eject has priority over toggle
        if (ejectButton) {
            ejectIntake();
            return;
        }

        if (toggleButton && !isTogglePressed) {
            intakeOn = !intakeOn;
        }
        isTogglePressed = toggleButton;

        if (intakeOn) {
            intakeMotor.setPower(INTAKE_IN_POWER);
            intakeMotor2.setPower(INTAKE_IN_POWER);
        }
        else {
            intakeMotor.setPower(INTAKE_STOP_POWER);
            intakeMotor2.setPower(INTAKE_STOP_POWER);
        }

    }

    public void ejectIntake() {
        intakeMotor.setPower(INTAKE_OUT_POWER);
        intakeMotor2.setPower(INTAKE_OUT_POWER);
    }
}
