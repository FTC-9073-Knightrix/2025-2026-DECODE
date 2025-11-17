package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, 45.7, Math.toRadians(127)))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135)) // shooting 1st ball
                // make it wait here
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(90)), Math.toRadians(90)) // Added missing closing parenthesis and tangent
                .strafeToLinearHeading(new Vector2d(-12, 50), Math.toRadians(90)) // goes for closest row and intakes it
                //wait for 3 seconds
                // .turn(Math.toRadians(60)) lets say u want to miss the ball
                .splineToLinearHeading(new Pose2d (-10, 50, Math.toRadians(90)), Math.toRadians(135)) // goes for closest row and intakes it
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135)) // shooting ball 2nd time
                //wait for 3 seconds
                .splineToLinearHeading(new Pose2d(14, 30, Math.toRadians(90)), Math.toRadians(90)) // goes for middle row
                .strafeToLinearHeading(new Vector2d(14, 50), Math.toRadians(90)) // intakes middle row
                .splineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(90)) // shooting ball 3rd time
                //wait for 3 seconds
                // go for last row (farthest row)

                .splineToLinearHeading(new Pose2d(38, 30, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38, 50), Math.toRadians(90))  // intakes all of the last row
                // return to shooting position
                .splineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(90)) // shooting ball 4th time
                // wait for 3 seconds
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}