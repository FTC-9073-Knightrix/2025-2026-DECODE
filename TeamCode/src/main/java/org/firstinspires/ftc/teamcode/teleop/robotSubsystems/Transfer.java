package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer {

    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public DcMotor transferMotor;
    public DistanceSensor transferDistanceSensor;
    private final double TRANSFER_IN_POWER = 1.0;
    private final double TRANSFER_OUT_POWER = -1.0;
    private final double TRANSFER_STOP_POWER = 0.0;

    boolean transferActive = false;
    boolean previousIntakeToggle = false;
    boolean forceStopTransfer = false;

    private DetectedColor currentColor = DetectedColor.NONE;

    public void init(HardwareMap hw) {
        transferMotor = hw.get(DcMotor.class, "transfer");
        transferDistanceSensor = hw.get(DistanceSensor.class, "transferDistanceSensor");
    }

    public void runTransferWithAutomaticStop(boolean transferToggle) {
        if (forceStopTransfer) {
            runTransferStop();
            return;
        }

        if (transferToggle && !previousIntakeToggle) {
            transferActive = !transferActive;
        }
        previousIntakeToggle = transferToggle;

        double distance = transferDistanceSensor.getDistance(DistanceUnit.CM); // in cm
        if (transferActive) {
            if (distance < 5.0) { // if an object is detected within 5 cm
                runTransferStop();
                forceStopTransfer = true;
                return;
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

    public DetectedColor getDetectedColor() {
        return currentColor;
    }
}
