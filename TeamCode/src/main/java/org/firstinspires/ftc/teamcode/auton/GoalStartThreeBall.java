package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Goal Start Three Ball", group="Autonomous")
public class GoalStartThreeBall extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Drive back to pick up third ball
        Action driveBackFromGoal = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-20, 0), 0, maxSpeedConstraint, maxAccelConstraint)
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
                            // DRIVE BACK AND SPIN UP SHOOTER SIMULTANEOUSLY
                            new ParallelAction(
                                driveBackFromGoal
                            )
                    )
            );
        }

    }
}
