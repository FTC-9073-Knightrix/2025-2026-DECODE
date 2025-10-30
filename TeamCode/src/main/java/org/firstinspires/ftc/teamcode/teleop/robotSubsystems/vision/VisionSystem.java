package org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSystem {
    public WebcamName webCam;
    public static String goalTagSequence = "NONE";
    VisionPortal visionPortal;

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

    public void init(HardwareMap hwMap) {
        webCam = hwMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webCam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    public boolean isDetectingAGoalTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            if (tag.id == AprilTagEnums.BLUE_GOAL.getId()
            || tag.id == AprilTagEnums.RED_GOAL.getId()) {;
                return true;
            }
        }
        return false;
    }

    public boolean isDetectingAnObeliskTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection tag: tagProcessor.getDetections()) {
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    return true;
                }
            }
        }
        return false;
    }

    public void scanGoalTagSequence() {
        AprilTagDetection tag;
        for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                tag = detectedTag;
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                    goalTagSequence = AprilTagEnums.OBELISK_TAG_21.getDescription();
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                    goalTagSequence = AprilTagEnums.OBELISK_TAG_22.getDescription();
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    goalTagSequence = AprilTagEnums.OBELISK_TAG_23.getDescription();
                }
                break;
            }
        }
    }

    /*
     * Bearing is the angle to the tag relative to the camera's forward direction.
     * Returns Bearing in Degrees
     */
    public double getTagBearing() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.bearing;
        }
        return 0;
    }

    /**
     * Gets the forward horizontal distance to the detected AprilTag.
     * This represents the forward horizontal distance from the camera's center axis to the tag.
     * Uses the Y coordinate from the FTC pose, which is the horizontal displacement.
     *
     * @return The forward horizontal distance to the first detected tag in distance units (typically inches),
     *         or -1 if no tags are detected
     */
    public double getTagHorizontalDistance() {

        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.y;
        }
        return -1;
    }

    public String getSequence() {
        return goalTagSequence;
    }
}
