package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Goal Start Three Ball", group="Autonomous")
public class GoalStartThreeBall extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Drive back to pick up third ball
        Action driveBackFromGoal = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-20, 0), 0, maxSpeedConstraint, maxAcceleration)
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
        }

    }
}
