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
                .strafeToLinearHeading(new Vector2d(-15, 15), Math.toRadians(135)) // shooting 1st ball
                // make it wait here
                .waitSeconds(2.5)

                .turn(Math.toRadians(-135 + 90))
                .setTangent(Math.toRadians(75))
                .strafeToLinearHeading(new Vector2d(-10, 54), Math.toRadians(90)) // Added missing closing parenthesis and tangent
                //wait for 3 seconds
                // .turn(Math.toRadians(60)) lets say u want to miss the ball
                .strafeToLinearHeading(new Vector2d(-15, 15), Math.toRadians(135)) // shooting ball 2nd time
                .waitSeconds(2.5)
                //wait for 3 seconds
                .strafeToLinearHeading(new Vector2d(1, 20), Math.toRadians(90)) // intakes middle row
                .splineToConstantHeading(new Vector2d(14, 50), Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-15, 15, Math.toRadians(135)), Math.toRadians(195)) // shooting ball 3rd time
                .waitSeconds(2.5)
                //wait for 3 seconds
                // go for last row (farthest row)
                .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36, 60), Math.toRadians(90))  // intakes all of the last row
                // return to shooting position
                .strafeToLinearHeading(new Vector2d(-15, 15), Math.toRadians(135)) // shooting ball 4th time
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(0, 45), Math.toRadians(90)) // park in the middle
                // wait for 3 seconds
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}