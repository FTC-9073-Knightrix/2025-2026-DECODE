package org.firstinspires.ftc.teamcode.teleop.mainRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.RGBLights;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.Transfer;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.drivetrain.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision.VisionSystem;

public abstract class RobotBaseHwMap extends OpMode {
    // ---------------Hardware Map and Variables------------
    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();
    VisionSystem vision = new VisionSystem();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    Transfer transfer = new Transfer();
    RGBLights lights = new RGBLights();

    @Override
    public void init() {
        // Initializing hardware mechanism classes
        drive.init(hardwareMap);
        vision.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        lights.init(hardwareMap);
    }
}
