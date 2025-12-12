package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name="Far Six Ball Red", group="Autonomous")
public class FarSixBallRed extends AutonMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        beginPose = new Pose2d(62, 14, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        AutonActions autonActions = new AutonActions(hardwareMap);

        // Drive back to pick up third ball
        Vector2d shootPos = new Vector2d(56, 14);
        double shootHeading = Math.toRadians(159);

        Action turnToGoal = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();


        // Drive into third row of balls
        Action driveToThirdRow = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(41, 24), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(41, 50))
                .build();

        Action driveToShoot = drive.actionBuilder(new Pose2d(41, 53.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(54, 14), Math.toRadians(157), minVelConstraint)
                .build();

        // Drive into corner of balls
        Action intakeCornerBalls = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(59, 55), Math.toRadians(85))
                .build();

        Action leaveLaunchZone = drive.actionBuilder(new Pose2d(shootPos, shootHeading))
                .strafeToLinearHeading(new Vector2d(61, 38), Math.toRadians(90))
                .build();

        Action Shoot3Balls = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToFarShot()
                ),
                // STOP INTAKE AND TRANSFER
                new ParallelAction(
                        autonActions.stopIntake(),
                        autonActions.stopTransfer()
                )
        );

        Action Shoot3Balls2 = new SequentialAction(
                // SHOOT 3 BALLS
                new ParallelAction(
                        autonActions.runIntake(),
                        autonActions.runTransferToFarShot()
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
                            // TURN TOWARD GOAL AND SPIN UP SHOOTER SIMULTANEOUSLY
                            new ParallelAction(
                                    turnToGoal,
                                    autonActions.setHoodToFarShot(),
                                    autonActions.spinShooterToFarShotVelocity()
                            ),
                            Shoot3Balls,
                            new ParallelAction(
                                    driveToThirdRow,
                                    autonActions.runIntake()
                            ),
                            driveToShoot,
                            Shoot3Balls2,
                            new ParallelAction(
                                    leaveLaunchZone,
                                    autonActions.stopIntake(),
                                    autonActions.stopTransfer(),
                                    autonActions.stopOuttake()
                            )
                    )
            );
        }
    }
}