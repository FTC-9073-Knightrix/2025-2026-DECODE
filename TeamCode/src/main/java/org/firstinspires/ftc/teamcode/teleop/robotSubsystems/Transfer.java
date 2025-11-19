package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {
    public DcMotor transferMotor;
    public DistanceSensor transferDistanceSensor;
    private boolean forceStopTransfer;
    private boolean transferActive;
    private boolean isTogglePressed;

    private final double TRANSFER_IN_POWER = 1.0;
    private final double TRANSFER_OUT_POWER = -1.0;
    private final double TRANSFER_STOP_POWER = 0.0;

    public void init(HardwareMap hw) {
        transferMotor = hw.get(DcMotor.class, "transfer");
        transferDistanceSensor = hw.get(DistanceSensor.class, "transferDistanceSensor");

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forceStopTransfer = false;
        transferActive = false;
        isTogglePressed = false;
    }

    public void runTransferWithAutomaticStop(boolean toggleButton) {
        // handle toggle on the rising edge first so the operator can re-enable even after an automatic stop
        if (toggleButton && !isTogglePressed) {
            transferActive = !transferActive;
            // if operator explicitly turns transfer back on, clear the forced stop
            if (transferActive) {
                forceStopTransfer = false;
            }
        }
        isTogglePressed = toggleButton;

        if (forceStopTransfer) {
            runTransferStop();
            return;
        }

        double distance = transferDistanceSensor.getDistance(DistanceUnit.CM); // in cm
        // make sensor reading robust: if invalid, treat as "no object" (very far)
        if (Double.isNaN(distance) || distance <= 0) {
            distance = Double.POSITIVE_INFINITY;
        }

        if (transferActive) {
            if (distance < 5.0) { // if an object is detected within 5 cm
                runTransferStop();
                forceStopTransfer = true;
            } else {
                runTransferIn();
            }
        } else {
            runTransferStop();
        }
    }

    public void runTransferIn() {
        transferMotor.setPower(TRANSFER_IN_POWER);
        forceStopTransfer = false;
    }

    public void runTransferOut() {
        transferMotor.setPower(TRANSFER_OUT_POWER);
    }

    public void runTransferStop() {
        transferMotor.setPower(TRANSFER_STOP_POWER);
    }
}
