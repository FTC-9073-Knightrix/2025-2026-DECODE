package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Twelve Ball Red", group="Autonomous")
public class TwelveBallRed extends AutonMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        beginPose = new Pose2d(-56, 45.7, Math.toRadians(127));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        AutonActions autonActions = new AutonActions(hardwareMap);

        // Drive back to pick up third ball
        Vector2d shootPos = new Vector2d(-15, 15);
        double shootHeading = Math.toRadians(135);
        Action driveBackFromGoal = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(shootPos, shootHeading, maxSpeedConstraint, maxAccelConstraint)
                .build();

        // Drive into first row of balls
        Action driveToFirstRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .turn(Math.toRadians(-45))
                .setTangent(Math.toRadians(75))
                .splineToLinearHeading(new Pose2d(-11, 54, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action driveBackToShoot1 = drive.actionBuilder(new Pose2d(-11, 54, Math.toRadians(90)))
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();
        // Drive into second row of balls
        Action driveToSecondRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(1, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(14, 50), Math.toRadians(90))
                .build();

        Action driveBackToShoot2 = drive.actionBuilder(new Pose2d(14, 50, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(shootPos, shootHeading), 195)
                .build();

        // Drive into third row of balls
        Action driveToThirdRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36, 50), Math.toRadians(90))
                .build();
        Action driveBackToShoot3 = drive.actionBuilder(new Pose2d(36, 50, Math.toRadians(90)))
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();

        // Drive to the gate
        Action driveToGate = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(0, 45), Math.toRadians(90))
                .build();

        Action Shoot3Balls = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransfer()
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
                                autonActions.stopOuttake(),
                                autonActions.runIntake(),
                                autonActions.runTransferUntilBallDetected(),
                                driveToFirstRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                driveBackToShoot1,
                                autonActions.stopIntake(),
                                autonActions.stopTransfer(),
                                autonActions.spinShooterToMidShotVelocity()
                            ),
                            Shoot3Balls,
                            // DRIVE TO SECOND ROW WHILE INTAKING
                            new ParallelAction(
                                autonActions.stopOuttake(),
                                autonActions.runIntake(),
                                autonActions.runTransferUntilBallDetected(),
                                driveToSecondRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                driveBackToShoot2,
                                autonActions.stopIntake(),
                                autonActions.stopTransfer(),
                                autonActions.spinShooterToMidShotVelocity()
                            ),
                            Shoot3Balls,
                            // DRIVE TO THIRD ROW WHILE INTAKING
                            new ParallelAction(
                                autonActions.stopOuttake(),
                                autonActions.runIntake(),
                                autonActions.runTransferUntilBallDetected(),
                                driveToThirdRow
                            ),
                            // DRIVE BACK TO SHOOTING POSITION
                            new ParallelAction(
                                driveBackToShoot3,
                                autonActions.stopIntake(),
                                autonActions.stopTransfer(),
                                autonActions.spinShooterToMidShotVelocity()
                            ),
                            Shoot3Balls,
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
