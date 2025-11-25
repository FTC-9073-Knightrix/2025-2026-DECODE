package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {
    public DcMotor transferMotor;
    public DistanceSensor transferDistanceSensor;
    public boolean transferActive;
    private boolean isTogglePressed;

    private final double TRANSFER_IN_POWER = 1.0;
    private final double TRANSFER_OUT_POWER = -1.0;
    private final double TRANSFER_STOP_POWER = 0.0;

    public void init(HardwareMap hw) {
        transferMotor = hw.get(DcMotor.class, "transfer");
        transferDistanceSensor = hw.get(DistanceSensor.class, "transferDistanceSensor");

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferActive = false;
        isTogglePressed = false;
    }

    public void runTransferWithAutomaticStop(boolean toggleButton) {
        // handle toggle on the rising edge first so the operator can re-enable even after an automatic stop
        if (toggleButton && !isTogglePressed) {
            transferActive = !transferActive;
        }
        isTogglePressed = toggleButton;

        double distance = transferDistanceSensor.getDistance(DistanceUnit.CM); // in cm

        if (transferActive) {
            if (distance < 2.0) { // if an object is detected within 2 cm
                runTransferStop();
            } else {
                runTransferIn();
            }
        } else {
            runTransferStop();
        }
    }

    public void runTransferIn() {
        transferMotor.setPower(TRANSFER_IN_POWER);
    }

    public void runTransferOut() {
        transferMotor.setPower(TRANSFER_OUT_POWER);
    }

    public void runTransferStop() {
        transferMotor.setPower(TRANSFER_STOP_POWER);
    }
}
