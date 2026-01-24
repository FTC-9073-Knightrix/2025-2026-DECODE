package org.firstinspires.ftc.teamcode.teleop.robotSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    // 28 ticks per revolution
    public DcMotor turretMotor;

    public void init(HardwareMap hw){
        turretMotor = hw.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double convertAngleToTicks(double angle) {
        // Assuming 1 full rotation (360 degrees) equals 1440 ticks
        return (angle / 360.0) * 1440.0;
    }

    private double convertTicksToTurretAngle(double ticks) {
        // 28 ticks per revolution of pinion
        // 1 : 4 gear ratio of pinion to turret
        // Therefore, 1 full rotation of turret (360 degrees) = 4 * 28 = 112 ticks
        return (ticks / 1440.0) * 360.0;
    }

    // TODO
    private double getDegreesOff(double targetAngle) {
        return 0;
    }


}
