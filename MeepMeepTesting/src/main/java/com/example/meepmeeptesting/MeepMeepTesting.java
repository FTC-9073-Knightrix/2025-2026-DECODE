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
        AllianceColor color = AllianceColor.RED; // Change color accordingly
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
// FAR THREE BALL RED
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 14, Math.toRadians(180)))
//                .strafeToLinearHeading(new Vector2d(54, 16), Math.toRadians(160))
//                .strafeToLinearHeading(new Vector2d(52, 56.5), Math.toRadians(70))
//                .strafeToLinearHeading(new Vector2d(58, 59), Math.toRadians(75))
//                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(60.5, 60.5), Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(54, 16), Math.toRadians(160))
//                .strafeToLinearHeading(new Vector2d(59, 38), Math.toRadians(90))
//
//                .build());


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-58, (color == AllianceColor.RED) ? 44 : -44, Math.toRadians((color == AllianceColor.RED) ? 127 : -127)))
                .strafeToLinearHeading(new Vector2d(-11, (color == AllianceColor.RED) ? 17 : -17), Math.toRadians((color == AllianceColor.RED) ? 140 : -140)) // shooting 1st ball
                // make it wait here
                .waitSeconds(2.5)

                .turnTo(Math.toRadians((color == AllianceColor.RED) ? (90) : -(90)))
                .strafeToConstantHeading(new Vector2d(-11, 50)) // Added missing closing parenthesis and tangent

                // open the gate
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-2, 51, Math.toRadians(180)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-11, (color == AllianceColor.RED) ? 17 : -17), Math.toRadians((color == AllianceColor.RED) ? 140 : -135)) // shooting 1st ball

                .strafeToLinearHeading(new Vector2d(12.5, (color == AllianceColor.RED) ? 26 : -26), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // intakes middle row
                .strafeToConstantHeading(new Vector2d(12.5, 50)) // Added missing closing parenthesis and tangent
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-11, (color == AllianceColor.RED) ? 17 : -17, Math.toRadians((color == AllianceColor.RED) ? 140 : -140)), Math.toRadians(220)) // shooting 1st ball

                .strafeToLinearHeading(new Vector2d(36, (color == AllianceColor.RED) ? 26 : -26), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // intakes middle row
                .strafeToConstantHeading(new Vector2d(36, 50)) // Added missing closing parenthesis and tangent
                .strafeToLinearHeading(new Vector2d(12.5, (color == AllianceColor.RED) ? 26 : -26), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // intakes middle row

//                .setTangent(Math.toRadians((color == AllianceColor.RED) ? 270 : -270))
//                .splineToLinearHeading(new Pose2d(-15, (color == AllianceColor.RED) ? 15 : -15, Math.toRadians((color == AllianceColor.RED) ? 135 : -135)), Math.toRadians((color == AllianceColor.RED) ? 195 : -195)) // shooting ball 3rd time
//                .waitSeconds(2.5)
//                //wait for 3 seconds
//                // go for last row (farthest row)
//                .strafeToLinearHeading(new Vector2d(36, (color == AllianceColor.RED) ? 30 : -30), Math.toRadians((color == AllianceColor.RED) ? 90 : -90))
//                .strafeToLinearHeading(new Vector2d(36, (color == AllianceColor.RED) ? 60 : -60), Math.toRadians((color == AllianceColor.RED) ? 90 : -90))  // intakes all of the last row
//                // return to shooting position
//                .strafeToLinearHeading(new Vector2d(-15, (color == AllianceColor.RED) ? 15 : -15), Math.toRadians((color == AllianceColor.RED) ? 135 : -135)) // shooting ball 4th time
//                .waitSeconds(2.5)
//                .strafeToLinearHeading(new Vector2d(0, (color == AllianceColor.RED) ? 45 : -45), Math.toRadians((color == AllianceColor.RED) ? 90 : -90)) // park in the middle
//                // wait for 3 seconds
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}