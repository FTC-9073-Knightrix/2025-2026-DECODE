package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous (name="LEAVE", group="Autonomous")
public class Leave extends AutonMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        beginPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        Action leaveLaunchZone = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToConstantHeading(new Vector2d(20, 0))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Waiting for Start", "Press Play to start");
            telemetry.update();
        }
        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) {
                telemetry.addData("Auton Stopped", "Stop Requested");
                telemetry.update();
                return;
            }
            telemetry.addData("Auton Elapsed Time", getRuntime());
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(
                            leaveLaunchZone
                    )
            );
        }
    }
}