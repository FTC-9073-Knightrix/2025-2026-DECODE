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

@Autonomous(name = "Pedro Close 12 Red", group = "Pedro Autonomous")
@Configurable // Panels
public class Close12Red extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.481, 131.668, Math.toRadians(37)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0; // Update autonomous state machine
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

        public PathChain DriveToShootPreload;
        public double Wait5;
        public PathChain Path2;
        public PathChain Path3;
        public double Wait6;
        public PathChain Path4;
        public PathChain Path7;
        public double Wait8;
        public PathChain Path9;
        public PathChain Path10;
        public double Wait11;

        public Paths(Follower follower) {
            DriveToShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(117.481, 131.668), new Pose(89.249, 83.288))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(48))
                    .build();

            Wait5 = 2000;

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(89.249, 83.288), new Pose(128.000, 83.428))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0), 0.2)
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.000, 83.428), new Pose(89.389, 83.149))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            Wait6 = 2000;

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.389, 83.149),
                                    new Pose(85.351, 57.069),
                                    new Pose(95.320, 59.379),
                                    new Pose(132.000, 59.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0), 0.3)
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.000, 59.400),
                                    new Pose(91.673, 59.014),
                                    new Pose(89.363, 83.209)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            Wait8 = 2000;

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.363, 83.209),
                                    new Pose(80.487, 27.800),
                                    new Pose(93.010, 35.670),
                                    new Pose(132.000, 35.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0), 0.3)
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(132.000, 35.400), new Pose(89.485, 108.255))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
                    .build();

            Wait11 = 2000;
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Drive to shooting position
                follower.followPath(paths.DriveToShootPreload);
                setPathState(1);
                break;
            case 1:
                // Wait for path to complete, then wait before shooting
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                // Wait timer for shooting preload
                if (pathTimer.getElapsedTimeSeconds() > paths.Wait5 / 1000.0) {
                    /* Shoot Preload Here */
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;
            case 3:
                // Drive to first sample
                if (!follower.isBusy()) {
                    /* Intake first sample */
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;
            case 4:
                // Return to shooting position with first sample
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                // Wait timer before shooting first sample
                if (pathTimer.getElapsedTimeSeconds() > paths.Wait6 / 1000.0) {
                    /* Shoot first sample here */
                    follower.followPath(paths.Path4);
                    setPathState(6);
                }
                break;
            case 6:
                // Drive to second sample
                if (!follower.isBusy()) {
                    /* Intake second sample */
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;
            case 7:
                // Return to shooting position with second sample
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                // Wait timer before shooting second sample
                if (pathTimer.getElapsedTimeSeconds() > paths.Wait8 / 1000.0) {
                    /* Shoot second sample here */
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;
            case 9:
                // Drive to third sample
                if (!follower.isBusy()) {
                    /* Intake third sample */
                    follower.followPath(paths.Path10);
                    setPathState(10);
                }
                break;
            case 10:
                // Return to final position
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                // Wait timer at final position
                if (pathTimer.getElapsedTimeSeconds() > paths.Wait11 / 1000.0) {
                    /* Shoot third sample here or park */
                    setPathState(-1); // End autonomous
                }
                break;
            case -1:
                // End of autonomous
                RobotStaticVariables.END_OF_AUTO_POSITION = follower.getPose();
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
