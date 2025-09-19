package org.firstinspires.ftc.teamcode.teleop.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "April Tag Auto Align  Test")
public class AprilTagAutoAlign extends OpMode {

    public WebcamName webCam;
    public TeleOpMecanumDrive drive = new TeleOpMecanumDrive();


    // Initialize the AprilTag processor which is the actual thing that does the vision processing
    // You can optionally configure parameters in the builder
    // For example, you can set the tag size, camera intrinsics, etc.
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

    // initializes the vision portal which is what we see on the driver station
    VisionPortal visionPortal;

    @Override
    public void init() {
        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webCam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        drive.init(hardwareMap);

    }
    @Override
    public void loop() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double leftY = -gamepad1.left_stick_y / 3;
        double leftX = gamepad1.left_stick_x / 3;
        double rightX = gamepad1.right_stick_x * .4;
        boolean yButton = gamepad1.y;

        // if there is at least one detection
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            double x = tag.ftcPose.x;
            double y = tag.ftcPose.y;
            double z = tag.ftcPose.z;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Tag Pose", tag.ftcPose);
            telemetry.addData("bearing", tag.ftcPose.bearing);
            telemetry.addData("x", "%.2f", x); // change format for more decimal places if needed
            telemetry.addData("y", "%.2f", y);
            telemetry.addData("z", "%.2f", z);

            // distances 2d and 3d
            double distance2d = Math.hypot(x, y);
            double distance3d = Math.hypot(distance2d, z);
            telemetry.addData("distance2d", "%.2f", distance2d);
            telemetry.addData("distance3d", "%.2f", distance3d);

            drive.autoAlignToTag(Math.toRadians(tag.ftcPose.bearing), rb, lb, leftY, leftX);
        }
        else {
            drive.runMecanumDrive(lb , rb, leftY, leftX, rightX, yButton);
        }

        telemetry.update();
        // Save more CPU resources when camera is no longer needed.
    }
}
