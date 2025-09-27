package org.firstinspires.ftc.teamcode.teleop.main;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.VisionSystem;

public abstract class TeleOpHardwareMap extends OpMode {
    // ---------------Hardware Map and Variables------------
    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();
    VisionSystem vision = new VisionSystem();
    @Override
    public void init() {

        drive.init(hardwareMap);
        vision.init(hardwareMap);

    }
}
