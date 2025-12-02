package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

@TeleOp(name="Basic Color Detection")
public class ColorDetectOpMode extends LinearOpMode {

    OpenCvCamera camera;
    ColorPipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        pipeline = new ColorPipeline();
        camera.setPipeline(pipeline);

        // openCameraDeviceAsync requires an AsyncCameraOpenListener with onOpened/onError methods
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // send an error to telemetry so users can see why camera failed to open
                telemetry.addData("Camera error", "code=%d", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Camera starting...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            String detected = pipeline.getDetectedColor();

            // show raw average RGB values for tuning
            telemetry.addData("R/G/B", "%.0f / %.0f / %.0f",
                    pipeline.getAvgRed(), pipeline.getAvgGreen(), pipeline.getAvgBlue());

            if (detected.equals("RED")) {
                telemetry.addLine("trash");
            } else if (detected.equals("GREEN")) {
                telemetry.addLine("artifacts");
            } else {
                telemetry.addLine("No strong color detected.");
            }

            telemetry.update();
        }
    }


    // ---------------------- PIPELINE --------------------------
    public static class ColorPipeline extends OpenCvPipeline {

        private String detectedColor = "NONE";
        // store the last average channel values so the OpMode can read them
        private double lastBlue = 0;
        private double lastGreen = 0;
        private double lastRed = 0;

        public String getDetectedColor() {
            return detectedColor;
        }

        public double getAvgBlue() { return lastBlue; }
        public double getAvgGreen() { return lastGreen; }
        public double getAvgRed() { return lastRed; }

        @Override
        public Mat processFrame(Mat input) {

            Scalar avg = Core.mean(input);

            double blue = avg.val[0];
            double green = avg.val[1];
            double red = avg.val[2];

            // store values for external access
            lastBlue = blue;
            lastGreen = green;
            lastRed = red;

            if (red > green && red > blue && red > 80) {
                detectedColor = "RED";
            } else if (green > red && green > blue && green > 80) {
                detectedColor = "GREEN";
            } else {
                detectedColor = "NONE";
            }

            return input;
        }
    }
}