package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Twelve Ball Blue", group="Autonomous")
public class TwelveBallBlue extends AutonMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        beginPose = new Pose2d(-58, -44, Math.toRadians(-127));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        AutonActions autonActions = new AutonActions(hardwareMap);

        // Drive back to pick up third ball
        Vector2d shootPos = new Vector2d(-12, -18);
        double shootHeading = Math.toRadians(-137.5);

        Action driveBackFromGoal = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .build();

        // Drive into first row of balls
        Action driveToFirstRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .turnTo(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-11, -43.5))
                .build();

        Action driveBackToShoot1 = drive.actionBuilder(new Pose2d(-11, -43.5, Math.toRadians(-90)))
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();
        // Drive into second row of balls
        Action driveToSecondRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(11, -26), Math.toRadians(-90), maxSpeedConstraint) // intakes middle row
                .strafeToConstantHeading(new Vector2d(11, -49.5))
                .build();

        Action driveBackToShoot2 = drive.actionBuilder(new Pose2d(11, -49.5, Math.toRadians(-90)))
//                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(shootPos, shootHeading), Math.toRadians(-220)) // shooting 1st ball
                .build();

        // Drive into third row of balls
        Action driveToThirdRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(34, -26), Math.toRadians(-90), maxSpeedConstraint)
                .strafeToConstantHeading(new Vector2d(34, -50))
                .build();

        Action driveBackToShoot3 = drive.actionBuilder(new Pose2d(34, -50, Math.toRadians(-90)))
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint)
                .build();

        // Drive to the gate
        Action driveToGate = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(-90), maxSpeedConstraint)
                .build();

        Action Shoot3Balls = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToMidShot()
                ),
                // STOP INTAKE AND TRANSFER
                new ParallelAction(
                        autonActions.stopIntake(),
                        autonActions.stopTransfer()
                )
        );
        Action Shoot3BallsTwo = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToMidShot()
                ),
                // STOP INTAKE AND TRANSFER
                new ParallelAction(
                        autonActions.stopIntake(),
                        autonActions.stopTransfer()
                )
        );
        Action Shoot3BallsThree = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToMidShot()
                ),
                // STOP INTAKE AND TRANSFER
                new ParallelAction(
                        autonActions.stopIntake(),
                        autonActions.stopTransfer()
                )
        );
        Action Shoot3BallsFour = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToMidShot()
                ),
                // STOP INTAKE AND TRANSFER
                new ParallelAction(
                        autonActions.stopIntake(),
                        autonActions.stopTransfer()
                )
        );

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
                                    driveBackFromGoal,
                                    autonActions.setHoodToMidShot(),
                                    autonActions.spinShooterToMidShotVelocity()
                            ),
                            Shoot3Balls,
                            // DRIVE TO FIRST ROW WHILE INTAKING
                            new ParallelAction(
                                    autonActions.runIntake(),
//                                autonActions.runTransferUntilBallDetected(),
                                    driveToFirstRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                    driveBackToShoot1
                            ),
                            Shoot3BallsTwo,
                            // DRIVE TO SECOND ROW WHILE INTAKING
                            new ParallelAction(
                                    autonActions.runIntake(),
//                                autonActions.runTransferUntilBallDetected(),
                                    driveToSecondRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                    driveBackToShoot2
                            ),
                            Shoot3BallsThree,
                            // DRIVE TO THIRD ROW WHILE INTAKING
                            new ParallelAction(
                                    autonActions.runIntake(),
//                                autonActions.runTransferUntilBallDetected(),
                                    driveToThirdRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                    driveBackToShoot3
                            ),
                            Shoot3BallsFour,
                            // DRIVE TO THE GATE
                            new ParallelAction(
                                    autonActions.stopOuttake(),
                                    driveToGate
                            )
                    )
            );
        }

    }
}