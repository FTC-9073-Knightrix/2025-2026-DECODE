package org.firstinspires.ftc.teamcode.auton;

import java.util.HashMap;
import java.util.Map;

public class BallOrderCalculator {
    public enum BallColor { 
        PURPLE, // p
        GREEN,  // g
        UNKNOWN
    }

    // the fixed order of the first three balls we start with
    private static final BallColor[] PRELOADED_ORDER = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};

    private static final Map<String, BallColor[]> MOTIF_TO_REQUIRED_SEQUENCE = new HashMap<>();

    static {
        MOTIF_TO_REQUIRED_SEQUENCE.put("GPP", new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE});
        MOTIF_TO_REQUIRED_SEQUENCE.put("PGP", new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE});
        MOTIF_TO_REQUIRED_SEQUENCE.put("PPG", new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});
    }

    // gets the 3-ball color sequence required by the motif
    public static BallColor[] getRequiredOrder(String motif) {
        BallColor[] order = MOTIF_TO_REQUIRED_SEQUENCE.get(motif);
        if (order == null) {
            // fallback if vision fails
            return MOTIF_TO_REQUIRED_SEQUENCE.get("PGP");
        }
        return order;
    }

    // checks if the ball should scored or missed based on detected color and motif
    public static boolean shouldMakeShot(BallColor detectedColor, int ballIndex, String motif) {
        BallColor[] required = getRequiredOrder(motif);
        // the pattern repeats every 3 balls
        int positionInCycle = ballIndex % 3;

        // compare detected color to the required pattern
        return detectedColor == required[positionInCycle];
    }

    // predicts the color of the ball we expect based only on the fixed preloaded order
    public static BallColor getExpectedColor(int ballIndex) {
        int positionInCycle = ballIndex % 3;
        return PRELOADED_ORDER[positionInCycle];
    }

    // gets the exact color needed to score for the current ball position
    public static BallColor getRequiredColor(int ballIndex, String motif) {
        BallColor[] required = getRequiredOrder(motif);
        int positionInCycle = ballIndex % 3;
        return required[positionInCycle];
    }

    // checks if the detected color matches the preloaded pattern
    public static boolean colorMatchesExpected(BallColor detectedColor, int ballIndex) {
        BallColor expected = getExpectedColor(ballIndex);
        return detectedColor == expected;
    }

    /* i dont really know what other way to do it so i just did strings
    but we can always change it to a boolean value or int idk
     */
    public static String getOrderDescription(String motif) {
        BallColor[] order = getRequiredOrder(motif);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < order.length; i++) {
            // convert enum to initial
            sb.append(order[i] == BallColor.PURPLE ? "P" : "G");
            if (i < order.length - 1) sb.append(" -> ");
        }
        return sb.toString();
    }

    // what our fixed preloaded order is
    public static String getPreloadedDescription() {
        return "P -> G -> P";
    }


    // TODO: 12/4/25 THIS METHOD IS COPILOT GENERATED SO WE MAY NEED TO ALTER PROPERLY IN THE FUTURE MAYBE 
    // shot plan for the first 3 preloaded balls (score/miss)
    public static String getShotPlanDescription(String motif) {
        BallColor[] required = getRequiredOrder(motif);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            // compare fixed ball order with motif requirement
            if (PRELOADED_ORDER[i] == required[i]) {
                sb.append("SCORE"); // match... GET SENDY WITH IT IF U FEEL ME LOL
            } else {
                sb.append("MISS");  // mismatch launch slowly
            }
            if (i < 2) sb.append(" | ");
        }
        return sb.toString();
    }
}