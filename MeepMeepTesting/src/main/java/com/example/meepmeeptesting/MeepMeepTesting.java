package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;

public class MeepMeepTesting {

    public enum AllianceColor {
        RED,
        BLUE
    }

    public static void main(String[] args) {
        AllianceColor color = AllianceColor.BLUE; // Change color accordingly
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, (color == AllianceColor.RED) ? 45.7 : -45.7, Math.toRadians((color == AllianceColor.RED) ? 127.5 : -127.5)))
                .strafeToLinearHeading(new Vector2d(-15, (color == AllianceColor.RED) ? 15 : -15), Math.toRadians((color == AllianceColor.RED) ? 135 : -135)) // shooting 1st ball
                // make it wait here
                .waitSeconds(2.5)

                .turn(Math.toRadians((color == AllianceColor.RED) ? (-135 + 90) : -(-135 + 90)))
                .setTangent(Math.toRadians((color == AllianceColor.RED) ? 75 : -75))
                .strafeToLinearHeading(new Vector2d(-10, (color == AllianceColor.RED) ? 54 : -54), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // Added missing closing parenthesis and tangent
                //wait for 3 seconds
                // .turn(Math.toRadians(60)) lets say u want to miss the ball
                .strafeToLinearHeading(new Vector2d(-15, (color == AllianceColor.RED) ? 15 : -15), Math.toRadians((color == AllianceColor.RED) ? 135 : -135)) // shooting ball 2nd time
                .waitSeconds(2.5)
                //wait for 3 seconds
                .strafeToLinearHeading(new Vector2d(1, (color == AllianceColor.RED) ? 20 : -20), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // intakes middle row
                .splineToConstantHeading(new Vector2d(14, (color == AllianceColor.RED) ? 50 : -50), Math.toRadians((color == AllianceColor.RED) ? 90 : -90))

                .setTangent(Math.toRadians((color == AllianceColor.RED) ? 270 : -270))
                .splineToLinearHeading(new Pose2d(-15, (color == AllianceColor.RED) ? 15 : -15, Math.toRadians((color == AllianceColor.RED) ? 135 : -135)), Math.toRadians((color == AllianceColor.RED) ? 195 : -195)) // shooting ball 3rd time
                .waitSeconds(2.5)
                //wait for 3 seconds
                // go for last row (farthest row)
                .strafeToLinearHeading(new Vector2d(36, (color == AllianceColor.RED) ? 30 : -30), Math.toRadians((color == AllianceColor.RED) ? 90 : -90))
                .strafeToLinearHeading(new Vector2d(36, (color == AllianceColor.RED) ? 60 : -60), Math.toRadians((color == AllianceColor.RED) ? 90 : -90))  // intakes all of the last row
                // return to shooting position
                .strafeToLinearHeading(new Vector2d(-15, (color == AllianceColor.RED) ? 15 : -15), Math.toRadians((color == AllianceColor.RED) ? 135 : -135)) // shooting ball 4th time
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(0, (color == AllianceColor.RED) ? 45 : -45), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // park in the middle
                // wait for 3 seconds
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}