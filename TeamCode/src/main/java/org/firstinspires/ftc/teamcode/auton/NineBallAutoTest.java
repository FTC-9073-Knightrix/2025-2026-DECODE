package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "Nine Ball Auto Test", group = "Autonomous")
public class NineBallAutoTest extends AutonBase {

    @Override
    public void runOpMode() throws InterruptedException {

        // INITIAL POSE — MATCHES MEEPMEEP
        TankDrive drive = new TankDrive(
                hardwareMap,
                new Pose2d(-56, 45.7, Math.toRadians(127))
        );

        super.runOpMode();
        Action path = drive.actionBuilder(new Pose2d(-56, 45.7, Math.toRadians(127)))

                // -------------------------
                // SHOOT BALL #1
                // -------------------------
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(2)

                // GO TO CLOSEST ROW (spline)
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, 50), Math.toRadians(90))
                .waitSeconds(3) // intake

                // GO BACK FOR SHOT #2
                .splineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(90)), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .waitSeconds(2)
                // -------------------------
                // SHOOT BALL #2
                // -------------------------

                // MIDDLE ROW
                .splineToLinearHeading(new Pose2d(14, 30, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(14, 50), Math.toRadians(90))
                .waitSeconds(3) // intake

                // RETURN FOR SHOT #3
                .splineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(90))
                .waitSeconds(2)
                // -------------------------
                // SHOOT BALL #3
                // -------------------------

                // LAST (FARTHEST) ROW
                .splineToLinearHeading(new Pose2d(38, 30, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38, 50), Math.toRadians(90))
                .waitSeconds(3) // intake

                // FINAL RETURN FOR SHOT #4
                .splineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(90))
                .waitSeconds(3)
                // -------------------------
                // SHOOT BALL #4
                // -------------------------

                .build();

        // WAIT FOR START
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Start Pose", "(-56, 45.7, 127°)");
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Running Auto");
            telemetry.update();

            Actions.runBlocking(path);

            telemetry.addData("Status", "AUTO COMPLETE");
            telemetry.update();
        }
    }
}