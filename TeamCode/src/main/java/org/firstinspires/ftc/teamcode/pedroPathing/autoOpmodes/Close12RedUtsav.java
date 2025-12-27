package org.firstinspires.ftc.teamcode.pedroPathing.autoOpmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close 12 Red Utsav", group = "Pedro Autonomous")
@Configurable // Panels
public class Close12RedUtsav extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain shootingcollected;
        public double Wait12;
        public PathChain gettingtoball;
        public PathChain Path3;
        public PathChain Path4;
        public double Wait6;
        public PathChain Path6;
        public double shootingrow1;
        public PathChain Path7;
        public double Wait8;
        public PathChain Path9;
        public PathChain Path10;
        public double Wait13;

        public Paths(Follower follower) {
            shootingcollected = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(109.000, 135.000),
                                    new Pose(90.000, 95.000),
                                    new Pose(96.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Wait12 = 1000;

            gettingtoball = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.000, 95.000),
                                    new Pose(98.837, 85.596),
                                    new Pose(104.749, 83.704)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.749, 83.704), new Pose(128.867, 83.704))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.867, 83.704), new Pose(96.000, 95.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Wait6 = 1000;

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.000, 95.000),
                                    new Pose(63.606, 49.182),
                                    new Pose(84.414, 52.493),
                                    new Pose(103.330, 58.877),
                                    new Pose(107.113, 55.094),
                                    new Pose(120.591, 56.985),
                                    new Pose(128.394, 56.512),
                                    new Pose(127.000, 63.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            shootingrow1 = 1000;

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.000, 63.000), new Pose(96.000, 95.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Wait8 = 1000;

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.000, 95.000),
                                    new Pose(96.000, 40.000),
                                    new Pose(96.000, 38.000),
                                    new Pose(96.000, 36.000),
                                    new Pose(96.000, 34.000),
                                    new Pose(96.000, 33.000),
                                    new Pose(96.000, 32.000),
                                    new Pose(135.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135.000, 36.000), new Pose(96.000, 95.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Wait13 = 1000;
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
