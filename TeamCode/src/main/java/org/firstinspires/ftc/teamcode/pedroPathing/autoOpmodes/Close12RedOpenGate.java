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

@Autonomous(name = "Pedro Close 12 Red Open Gate", group = "Pedro Autonomous")
@Configurable // Panels
public class Close12RedOpenGate extends OpMode {

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
        follower.setStartingPose(new Pose(111.837, 136.042, Math.toRadians(0)));

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

    @Override
    public void stop() {
        RobotStaticVariables.END_OF_AUTO_POSITION = follower.getPose();
    }

    public static class Paths {
        public double WaitTime;
        public PathChain DriveToShootPreload;
        public PathChain DriveToFirstTape;
        public PathChain OpenTheGate;
        public PathChain DriveBackFromGate;
        public PathChain DriveToSecondTape;
        public PathChain DriveBackFromSecondTape;
        public PathChain DriveToThirdTape;
        public PathChain DriveBackFromThirdTape;
        public PathChain DriveToFinalGatePosition;

        public Paths(Follower follower) {
            WaitTime = 3.5; // seconds
            DriveToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.838, 136.042),
                                    new Pose(88.471, 83.597)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DriveToFirstTape = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.471, 83.597),
                                    new Pose(127.176, 83.252)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            OpenTheGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.176, 83.252),
                                    new Pose(120.197, 76.214),
                                    new Pose(128.496, 73.681)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            DriveBackFromGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.496, 73.681),
                                    new Pose(88.597, 83.588)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            DriveToSecondTape = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.597, 83.588),
                                    new Pose(82.550, 54.147),
                                    new Pose(124.105, 59.408),
                                    new Pose(134.017, 58.420)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DriveBackFromSecondTape = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.017, 58.420),
                                    new Pose(103.218, 63.849),
                                    new Pose(88.555, 83.412)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DriveToThirdTape = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.555, 83.412),
                                    new Pose(86.824, 28.777),
                                    new Pose(104.597, 35.416),
                                    new Pose(133.479, 35.403)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DriveBackFromThirdTape = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.479, 35.403),
                                    new Pose(88.218, 83.655)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DriveToFinalGatePosition = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.218, 83.655),
                                    new Pose(118.798, 70.092)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case WAITING_TO_START:
                // Drive to shooting position
                follower.followPath(paths.DriveToShootPreload);
                // robot.runIntake();
                // robot.spinShooterForMidShot();
                setPathState(PathState.DRIVE_TO_SHOOT_PRELOAD);
                break;
            case DRIVE_TO_SHOOT_PRELOAD:
                // Wait for path to complete before shooting preload
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_PRELOAD);
                    // robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_FIRST_PRELOAD:
                // robot.runTransfer(); // needs to be called repeatedly to run transfer
                if (/*!robot.transferStillRunning() ||*/ pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToFirstTape);
                    // robot.resetTransfer();
                    // robot.stopTransfer();
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
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(paths.DriveBackFromGate);
                    setPathState(PathState.RETURN_TO_SHOOT_FIRST_THREE);
                }
                break;
            case RETURN_TO_SHOOT_FIRST_THREE:
                // Return to shooting position with first three artifacts
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_THREE);
                    // robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_FIRST_THREE:
                // robot.runTransfer(); // needs to be called repeatedly to run transfer
                if (/*!robot.transferStillRunning() ||*/ pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToSecondTape);
                    // robot.resetTransfer();
                    // robot.stopTransfer();
                    setPathState(PathState.DRIVE_TO_SECOND_TAPE);
                }
                break;
            case DRIVE_TO_SECOND_TAPE:
                // Drive to second sample
                if (!follower.isBusy()) {
                    /* Intake second sample */
                    follower.followPath(paths.DriveBackFromSecondTape);
                    setPathState(PathState.RETURN_TO_SHOOT_SECOND_THREE);
                }
                break;
            case RETURN_TO_SHOOT_SECOND_THREE:
                // Return to shooting position with second sample
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_THREE);
                    // robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_SECOND_THREE:
                // robot.runTransfer();
                if (/*!robot.transferStillRunning() ||*/ pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToThirdTape);
                    setPathState(PathState.DRIVE_TO_THIRD_TAPE);
                    // robot.resetTransfer();
                    // robot.stopTransfer();
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
                    // robot.initTransfer(robot.midShotTargetVelocityTicks);
                }
                break;
            case SHOOT_THIRD_THREE:
                // robot.runTransfer();
                if (/*!robot.transferStillRunning() ||*/ pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    /* Shoot third sample here or park */
                    setPathState(PathState.DRIVE_TO_FINAL_GATE_POSITION);
                    follower.followPath(paths.DriveToFinalGatePosition);
                    // robot.resetTransfer();
                    // robot.stopTransfer();
                    // robot.stopIntake();
                    // robot.stopShooter();
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

