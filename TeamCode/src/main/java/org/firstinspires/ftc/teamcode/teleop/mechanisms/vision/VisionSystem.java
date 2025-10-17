package org.firstinspires.ftc.teamcode.teleop.mechanisms.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSystem {
    public WebcamName webCam;
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


}
