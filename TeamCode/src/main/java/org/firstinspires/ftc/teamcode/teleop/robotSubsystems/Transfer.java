package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    public DcMotor transferMotor;
    private final double TRANSFER_IN_POWER = 0.5;
    private final double TRANSFER_OUT_POWER = -0.5;
    private final double TRANSFER_STOP_POWER = 0.0;

    public void init(HardwareMap hw) {
        transferMotor = hw.get(DcMotor.class, "transfer");
    }

    public void runTransferIn() {
        transferMotor.setPower(TRANSFER_IN_POWER);
    }

    public void runTransferOut() {
        transferMotor.setPower(TRANSFER_OUT_POWER);
    }
}
