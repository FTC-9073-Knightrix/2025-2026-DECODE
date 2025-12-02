package org.openftc.apriltag;

import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetection;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private final double tagSize;
    private final double fx, fy, cx, cy;

    // store the most recent detections
    private final List<AprilTagDetection> latestDetections = new ArrayList<>();

    public AprilTagDetectionPipeline(double tagSize, double fx, double fy, double cx, double cy) {
        this.tagSize = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

            Class<?> detectorClass = Class.forName("org.openftc.apriltag.AprilTagDetector");
            Object detector = detectorClass.getDeclaredConstructor().newInstance();

            java.lang.reflect.Method detectMethod = detectorClass.getMethod("detect", Mat.class);
            @SuppressWarnings("unchecked")
            List<AprilTagDetection> detections = (List<AprilTagDetection>) detectMethod.invoke(detector, gray);

            synchronized (latestDetections) {
                latestDetections.clear();
                if (detections != null) {
                    latestDetections.addAll(detections);
                }
            }

        } catch (ClassNotFoundException e) {
        } catch (Exception e) {
        }

        return input;
    }

    public List<AprilTagDetection> getLatestDetections() {
        synchronized (latestDetections) {
            return new ArrayList<>(latestDetections);
        }
    }
}

