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

import org.firstinspires.ftc.teamcode.RobotStaticVariables;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far 9 Red Delayed", group = "Pedro Autonomous")
@Configurable // Panels
public class Far9RedDelayed extends OpMode {

    public enum PathState {
        WAITING_TO_START,
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_FIRST_PRELOAD,
        DRIVE_TO_FIRST_TAPE,
        WAITING,
        RETURN_TO_SHOOT_FIRST_THREE,
        SHOOT_FIRST_THREE,
        DRIVE_TO_CORNER,
        RETURN_TO_SHOOT_CORNER,
        SHOOT_CORNER,
        DRIVE_TO_LEAVE
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
        follower.setStartingPose(new Pose(86.857, 8.990, Math.toRadians(90)));

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

        public double WaitTime = 3.5;
        public PathChain DriveToShootPreload;
        public PathChain DriveToFirstTape;
        public PathChain DriveBackFromFirstTape;
        public PathChain DriveToCorner;
        public PathChain DriveBackFromCorner;
        public PathChain DriveToLeave;

        public Paths(Follower follower) {
            DriveToShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.857, 8.990), new Pose(88.229, 16.152))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            DriveToFirstTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.229, 16.152),
                                    new Pose(80.305, 37.638),
                                    new Pose(110.933, 35.505),
                                    new Pose(134.705, 35.505)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0), 0.3)
                    .build();

            DriveBackFromFirstTape = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.705, 35.505), new Pose(88.381, 16.152))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            DriveToCorner = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.381, 16.152), new Pose(133.029, 12.038))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0), 0.4)
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.029, 12.038),
                                    new Pose(122.057, 18.133),
                                    new Pose(118.248, 10.514),
                                    new Pose(132.876, 9.905)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            DriveBackFromCorner = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(132.876, 9.905), new Pose(88.381, 16.305))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            DriveToLeave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.381, 16.305), new Pose(119.619, 10.819))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case WAITING_TO_START:
                // Drive to shooting position
                follower.followPath(paths.DriveToShootPreload);
                robot.runIntake();
                robot.spinShooterForFarShot();
                setPathState(PathState.DRIVE_TO_SHOOT_PRELOAD);
                break;
            case DRIVE_TO_SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 2) { // give 2 seconds for flywheel to spin up
                    setPathState(PathState.SHOOT_FIRST_PRELOAD);
                    robot.initTransfer(robot.farShotTargetVelocityTicks);
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
                    follower.followPath(paths.DriveBackFromFirstTape);
                    setPathState(PathState.RETURN_TO_SHOOT_FIRST_THREE);
                }
                break;
            case RETURN_TO_SHOOT_FIRST_THREE:
                // Return to shooting position with first three artifacts
                if (!follower.isBusy()) {
                    setPathState(PathState.WAITING);
                }
                break;
            case WAITING:
                if (pathTimer.getElapsedTimeSeconds() > 11) {
                    setPathState(PathState.SHOOT_FIRST_THREE);
                    robot.initTransfer(robot.farShotTargetVelocityTicks);
                }
            case SHOOT_FIRST_THREE:
                robot.runTransfer(); // needs to be called repeatedly to run transfer
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToCorner);
                    robot.resetTransfer();
                    robot.stopTransfer();
                    setPathState(PathState.DRIVE_TO_CORNER);
                }
                break;
            case DRIVE_TO_CORNER:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(paths.DriveBackFromCorner);
                    setPathState(PathState.RETURN_TO_SHOOT_CORNER);
                }
                break;
            case RETURN_TO_SHOOT_CORNER:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(PathState.SHOOT_CORNER);
                    robot.initTransfer(robot.farShotTargetVelocityTicks);
                }
                break;
            case SHOOT_CORNER:
                robot.runTransfer();
                if (!robot.transferStillRunning() || pathTimer.getElapsedTimeSeconds() > paths.WaitTime) {
                    follower.followPath(paths.DriveToLeave);
                    robot.resetTransfer();
                    robot.stopTransfer();
                    robot.stopIntake();
                    robot.stopShooter();
                    setPathState(PathState.DRIVE_TO_LEAVE);
                }
                break;
            case DRIVE_TO_LEAVE:
                // end of auto
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
