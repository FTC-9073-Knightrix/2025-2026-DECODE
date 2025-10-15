package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.mechanisms.TeleOpMecanumDrive;

import java.util.Locale;

@TeleOp(name = "mecanum TeleOp")
public class MecanumTest extends OpMode {
    TeleOpMecanumDrive drive = new TeleOpMecanumDrive();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x * .8;
        boolean yButton = gamepad1.y;
        drive.runManualMecanumDrive(rb, lb, leftY, leftX, rightX, yButton);

        telemetry.addData("FL Power", drive.frontLeftMotor.getPower());
        telemetry.addData("FR Power", drive.frontRightMotor.getPower());
        telemetry.addData("BL Power", drive.backLeftMotor.getPower());
        telemetry.addData("BR Power", drive.backRightMotor.getPower());
        telemetry.addData("finalSlowMode", drive.finalSlowMode);
        telemetry.addData("left bumper (fast)", gamepad1.left_bumper);
        telemetry.addData("right bumper (slow)", gamepad1.right_bumper);
        telemetry.addData("left stick y", -gamepad1.left_stick_y);
        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("y button (reset heading)", gamepad1.y);
//        telemetry.addData( "imu heading: ", String.format(Locale.US, "%.2f", drive.orientation.getYaw(AngleUnit.DEGREES)));
        telemetry.addData( "imu heading: ", String.format(Locale.US, "%.2f", drive.pinpoint.getHeading(AngleUnit.DEGREES)));
    }

}
