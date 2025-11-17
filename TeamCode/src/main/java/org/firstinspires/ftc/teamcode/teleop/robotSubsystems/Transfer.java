package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public DcMotor transferMotor;
    private final double TRANSFER_IN_POWER = 0.5;
    private final double TRANSFER_OUT_POWER = -0.5;
    private final double TRANSFER_STOP_POWER = 0.0;

    private DetectedColor currentColor = DetectedColor.NONE;

    public void init(HardwareMap hw) {
        transferMotor = hw.get(DcMotor.class, "transfer");
    }

    public void setDetectedColor(DetectedColor color) {
        this.currentColor = color;
    }

    private boolean isColorValid() {
        return currentColor == DetectedColor.GREEN || currentColor == DetectedColor.PURPLE;
    }

    public void runTransferIn() {
        if (isColorValid()) {
            transferMotor.setPower((TRANSFER_IN_POWER));
        } else {
            transferMotor.setPower(TRANSFER_STOP_POWER);
        }
    }

    public void runTransferOut() {

        transferMotor.setPower(TRANSFER_OUT_POWER);
    }

    public DetectedColor getDetectedColor() {
        return currentColor;
    }
}
