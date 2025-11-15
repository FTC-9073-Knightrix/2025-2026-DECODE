package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name= "Nine Ball Auto Test", group="Autonomous")
public class NineBallAutoTest extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {
        // initial pose for tankdrive by utsav das :))))
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(-56, 45, Math.toRadians(37)));
        super.runOpMode();

        // Build the trajectory action
        Action trajectoryAction = drive.actionBuilder(new Pose2d(-56, 45, Math.toRadians(37)))
                // move to initial shooting position
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(1) // wait at shooting position

                // first shot
                .strafeToLinearHeading(new Vector2d(-100, 30), Math.toRadians(90))
                .waitSeconds(3) // wait for shooting mechanism

                // go for closest row and intake
                .strafeToLinearHeading(new Vector2d(-10, 50), Math.toRadians(90))

                // return and shoot 2nd ball
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(3) // wait for shooting

                // go for middle row
                .strafeToLinearHeading(new Vector2d(14, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(14, 50), Math.toRadians(90))

                // return and shoot 3rd ball
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(3) // wait for shooting

                // go for last row (farthest row)
                .strafeToLinearHeading(new Vector2d(38, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38, 45), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38, 50), Math.toRadians(90))

                // return to shooting position and shoot 4th ball
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(3) // wait for final shooting
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for Start");
            telemetry.addData("Starting Pose", "(-56, 45, 37°)");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) {
                telemetry.addData("Auton Stopped", "Stop Requested");
                telemetry.update();
                return;
            }

            telemetry.addData("Status", "Running Nine Ball Auto");
            telemetry.update();

            Actions.runBlocking(trajectoryAction);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}