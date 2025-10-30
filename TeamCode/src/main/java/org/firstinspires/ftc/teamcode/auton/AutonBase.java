package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

import java.util.Arrays;

@Config
public abstract class AutonBase extends LinearOpMode {
    MecanumDrive drive;
    AutonIntake intake;

    protected Pose2d beginPose;
    protected VelConstraint maxSpeedConstraint;
    protected  VelConstraint baseVelConstraint;
    protected AccelConstraint maxAccelConstraint;

    @Override
    public void runOpMode() throws InterruptedException {
        // The beginPose can be changed to set the robot's starting position on the field.
        // We set it as (0, 0, 0), you would change the values to a field position
        // If you want to design autonomous paths using field coordinates use something like MeepMeep
        // However, stuff seemed to break last year so we just used relative coordinates from the (0, 0, 0)
        Pose2d beginPose = new Pose2d(0, 0, 0);

        // the inches per tick values come from measurements for the GoBilda 4-bar Pinpoint Odometry wheels, which FTC 9073 uses.
        // the calculation is 32mm wheel diameter, converted to inches = 1.25984251969 inch circumference
        // 1.26 in * pi = 3.958 in circumference
        // Assuming 2000 ticks per revolution from the GoBilda documentation,
        // 3.958 in / 2000 ticks = 0.001979 inches per tick
        double inPerTick = 0.00198;

        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        intake = new AutonIntake(hardwareMap);

        baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        maxSpeedConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(65.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        maxAccelConstraint = new ProfileAccelConstraint(-40.0, 70);
    }
}

