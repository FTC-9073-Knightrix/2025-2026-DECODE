package org.firstinspires.ftc.teamcode.pedroPathing.autoOpmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotStaticVariables;

@Autonomous(name = "Pedro Close 12 Blue Double Open Gate", group = "Pedro Autonomous")
@Configurable // Panels
public class Close12BlueDoubleOpenGate extends OpMode {

    public enum PathState {
        WAITING_TO_START,
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_FIRST_PRELOAD,
        DRIVE_TO_FIRST_TAPE,
        OPEN_THE_GATE,
        WAIT_WHILE_OPENING_GATE,
        RETURN_TO_SHOOT_FIRST_THREE,
        SHOOT_FIRST_THREE,
        DRIVE_TO_SECOND_TAPE,
        OPEN_THE_GATE_2,
        WAIT_WHILE_OPENING_GATE_2,
        RETURN_TO_SHOOT_SECOND_THREE,
        SHOOT_SECOND_THREE,
        DRIVE_TO_THIRD_TAPE,
        RETURN_TO_SHOOT_THIRD_THREE,
        SHOOT_THIRD_THREE,
        DRIVE_TO_FINAL_GATE_POSITION
    }

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    PedroRobot robot;

    @Override
    public void init() {
        robot = new PedroRobot(hardwareMap); // robot with all of our attachments
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.481, 131.668, Math.toRadians(37)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = PathState.WAITING_TO_START; // Update autonomous state machine
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public double WaitTime;
        public PathChain DriveToShootPreload;
        public PathChain DriveToFirstTape;
        public PathChain OpenTheGate;
        public PathChain DriveBackFromGate;
        public PathChain DriveToSecondTape;
        public PathChain OpenTheGate2;
        public PathChain DriveBackFromGate2;
        public PathChain DriveToThirdTape;
        public PathChain DriveBackFromThirdTape;
        public PathChain DriveToFinalGatePosition;

        public Paths(Follower follower) {
            WaitTime = 3.5; // seconds
            DriveToShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144 - 117.481, 131.668), new Pose(144 - 89.249, 83.288))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 37), Math.toRadians(180 - 48))
                    .build();

            DriveToFirstTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144 - 89.249, 83.288), new Pose(144 - 128.000, 83.428))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0), 0.2)

                    .build();

            OpenTheGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - 128.000, 83.428),
                                    new Pose(144 - 119.363, 79.257),
                                    new Pose(144 - 110.960, 72.191),
                                    new Pose(144 - 127.340, 75.0)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                    .setVelocityConstraint(10)
                    .build();

            DriveBackFromGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144 - 127.340, 75), new Pose(144 - 89.389, 83.149))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 48))
                    .build();


            DriveToSecondTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - 89.389, 83.149),
                                    new Pose(144 - 85.351, 57.069),
                                    new Pose(144 - 95.320, 59.379),
                                    new Pose(144 - 135.000, 59.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0), 0.3)
                    .build();

            DriveBackFromGate2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144 - 127.767, 69.899), // drive back from the gate instead
                                    new Pose(144 - 89.363, 83.209)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 48))
                    .build();


            DriveToThirdTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - 89.363, 83.209),
                                    new Pose(144 - 80.487, 27.800),
                                    new Pose(144 - 93.010, 35.670),
                                    new Pose(144 - 135.000, 35.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0), 0.3)
                    .build();

            DriveBackFromThirdTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144 - 134.000, 35.400), new Pose(144 - 89.389, 83.428))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 48))
                    .build();

            DriveToFinalGatePosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144 - 89.389, 83.428), new Pose(144 - 115.968, 69.999))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0))
                    .build();

            OpenTheGate2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - 135.000, 59.400),
                                    new Pose(144 - 111.801, 57.837),
                                    new Pose(144 - 127.767, 69.899)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(0))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case WAITING_TO_START:
                // Drive to shooting position
                follower.followPath(paths.DriveToShootPreload);
                robot.runIntake();
                robot.spinShooterForMidShot();
                setPathState(PathState.DRIVE_TO_SHOOT_PRELOAD);
                break;
            case DRIVE_TO_SHOOT_PRELOAD:
                // Wait for path to complete before shooting preload
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_PRELOAD);
                    robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_FIRST_PRELOAD:
                robot.runTransfer(); // needs to be called repeatedly to run transfer
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToFirstTape);
                    robot.resetTransfer();
                    robot.stopTransfer();
                    setPathState(PathState.DRIVE_TO_FIRST_TAPE);
                }
                break;
            case DRIVE_TO_FIRST_TAPE:
                // Drive to first tape mark
                if (!follower.isBusy()) {
                    follower.followPath(paths.OpenTheGate);
                    setPathState(PathState.OPEN_THE_GATE);
                }
                break;
            case OPEN_THE_GATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_WHILE_OPENING_GATE);
                }
                break;
            case WAIT_WHILE_OPENING_GATE:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(paths.DriveBackFromGate);
                    setPathState(PathState.RETURN_TO_SHOOT_FIRST_THREE);
                }
                break;
            case RETURN_TO_SHOOT_FIRST_THREE:
                // Return to shooting position with first three artifacts
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_THREE);
                    robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_FIRST_THREE:
                robot.runTransfer(); // needs to be called repeatedly to run transfer
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToSecondTape);
                    robot.resetTransfer();
                    robot.stopTransfer();
                    setPathState(PathState.DRIVE_TO_SECOND_TAPE);
                }
                break;
            case DRIVE_TO_SECOND_TAPE:
                // Drive to second sample
                if (!follower.isBusy()) {
                    /* Intake second sample */
                    follower.followPath(paths.OpenTheGate2);
                    setPathState(PathState.OPEN_THE_GATE_2);
                }
                break;
            case OPEN_THE_GATE_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_WHILE_OPENING_GATE_2);
                }
                break;
            case WAIT_WHILE_OPENING_GATE_2:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(paths.DriveBackFromGate2);
                    setPathState(PathState.RETURN_TO_SHOOT_SECOND_THREE);
                }
                break;
            case RETURN_TO_SHOOT_SECOND_THREE:
                // Return to shooting position with second sample
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_THREE);
                    robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_SECOND_THREE:
                robot.runTransfer();
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToThirdTape);
                    setPathState(PathState.DRIVE_TO_THIRD_TAPE);
                    robot.resetTransfer();
                    robot.stopTransfer();
                }
                break;
            case DRIVE_TO_THIRD_TAPE:
                // Drive to third sample
                if (!follower.isBusy()) {
                    /* Intake third sample */
                    follower.followPath(paths.DriveBackFromThirdTape);
                    setPathState(PathState.RETURN_TO_SHOOT_THIRD_THREE);
                }
                break;
            case RETURN_TO_SHOOT_THIRD_THREE:
                // Return to final position
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_THREE);
                    robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_THIRD_THREE:
                robot.runTransfer();
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    /* Shoot third sample here or park */
                    setPathState(PathState.DRIVE_TO_FINAL_GATE_POSITION);
                    follower.followPath(paths.DriveToFinalGatePosition);
                    robot.resetTransfer();
                    robot.stopTransfer();
                    robot.stopIntake();
                    robot.stopShooter();
                }
                break;
            case DRIVE_TO_FINAL_GATE_POSITION:
                // End of autonomous
                if (!follower.isBusy()) {
                    RobotStaticVariables.END_OF_AUTO_POSITION = follower.getPose();
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
