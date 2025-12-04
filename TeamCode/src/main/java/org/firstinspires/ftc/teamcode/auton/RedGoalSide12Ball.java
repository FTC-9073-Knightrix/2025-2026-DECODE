package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Red Goal Side 12 Ball", group="Autonomous")
public class RedGoalSide12Ball extends AutonMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        beginPose = new Pose2d(-58, 44, Math.toRadians(127));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        AutonActions autonActions = new AutonActions(hardwareMap);

        // Shooting position
        Vector2d shootPos = new Vector2d(-12, 18);
        double shootHeading = Math.toRadians(137.5);

        // ==================== APRILTAG DETECTION PHASE ====================
        while (!isStopRequested() && !opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : tagProcessor.getDetections()) {
                    if (detection.id >= 21 && detection.id <= 23) {
                        if (detection.id == 21) motif = "GPP";
                        else if (detection.id == 22) motif = "PGP";
                        else if (detection.id == 23) motif = "PPG";
                        break;
                    }
                }
            }

            telemetry.addData("Detected Motif", motif);
            telemetry.addData("Required Order", BallOrderCalculator.getOrderDescription(motif));
            telemetry.addData("Preloaded Order", BallOrderCalculator.getPreloadedDescription());
            telemetry.addData("Shot Plan", BallOrderCalculator.getShotPlanDescription(motif));
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
        }

        telemetry.addData("Final Motif", motif);
        telemetry.update();

        // ==================== COLLECT ALL BALLS ====================
        Action driveBackFromGoal = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .build();

        Action driveToFirstRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(-11, 43.5), Math.toRadians(90), maxSpeedConstraint)
                .build();

        Action driveBackToShoot1 = drive.actionBuilder(new Pose2d(-11, 43.5, Math.toRadians(90)))
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();

        Action driveToSecondRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(11, 49.5), Math.toRadians(90), maxSpeedConstraint)
                .build();

        Action driveBackToShoot2 = drive.actionBuilder(new Pose2d(11, 49.5, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(shootPos, shootHeading), Math.toRadians(220))
                .build();

        Action driveToThirdRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(34, 50), Math.toRadians(90), maxSpeedConstraint)
                .build();

        Action driveBackToShoot3 = drive.actionBuilder(new Pose2d(34, 50, Math.toRadians(90)))
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .build();

        Action driveToFourthRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(57, 50), Math.toRadians(90), maxSpeedConstraint)
                .build();

        Action driveBackToShoot4 = drive.actionBuilder(new Pose2d(57, 50, Math.toRadians(90)))
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .build();

        Action driveToGate = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(0, 20), Math.toRadians(90), maxSpeedConstraint)
                .build();

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;

            autonActions.resetBallCounter();

            telemetry.addData("Motif", motif);
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    driveBackFromGoal,
                                    autonActions.setHoodToMidShot(),
                                    autonActions.spinShooterToMidShotVelocity()
                            ),
                            autonActions.smartTransferMidShot(motif),

                            new ParallelAction(
                                    autonActions.runIntake(),
                                    driveToFirstRow
                            ),
                            driveBackToShoot1,
                            autonActions.smartTransferMidShot(motif),

                            new ParallelAction(
                                    autonActions.runIntake(),
                                    driveToSecondRow
                            ),
                            driveBackToShoot2,
                            autonActions.smartTransferMidShot(motif),

                            new ParallelAction(
                                    autonActions.runIntake(),
                                    driveToThirdRow
                            ),
                            driveBackToShoot3,
                            autonActions.smartTransferMidShot(motif),

                            new ParallelAction(
                                    autonActions.runIntake(),
                                    driveToFourthRow
                            ),
                            driveBackToShoot4,
                            autonActions.smartTransferMidShot(motif),

                            new ParallelAction(
                                    autonActions.stopOuttake(),
                                    driveToGate
                            )
                    )
            );
        }
    }
}
