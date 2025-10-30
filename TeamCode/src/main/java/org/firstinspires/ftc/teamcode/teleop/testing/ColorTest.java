package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Test")
public class ColorTest extends OpMode {
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    float gain = 22;
    NormalizedRGBA colors;
    enum GameColors {GREEN, PURPLE, NONE}
    GameColors colorMatch = GameColors.NONE;
    double minColorDistance = 7.0; // Minimum distance in cm to detect color

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        readColorData();
        getColorData();
        getColorMatch();
        telemetry.update();
    }
    public void readColorData() {
        // Get the normalized colors from the sensor
        colors = colorSensor.getNormalizedColors();
    }
    public void getColorData() {
        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            gain += 0.005F;
        } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            gain -= 0.005F;
        }

        // Show the gain value via telemetry
        telemetry.addData("Gain:", gain);
        colorSensor.setGain(gain);


        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Output the sensor data to telemetry
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);

        telemetry.addData("Color Match", colorMatch + "\n");
        telemetry.addData("Distance", "%.3f", distanceSensor.getDistance(DistanceUnit.CM));
    }

    public void getColorMatch() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (distance > minColorDistance) {
            colorMatch = GameColors.NONE;
            return;
        }

        if ((colors.green > 0.2 ) && (colors.green > colors.red) && (colors.green > colors.blue)) {
            colorMatch = GameColors.GREEN;
        }
        else if (colors.red > 0.2 && colors.blue > 0.2
                && colors.green < 0.2
                && colors.red > colors.green
                && colors.blue > colors.green )
        {
            colorMatch = GameColors.PURPLE;
        }
        else {
            colorMatch = GameColors.NONE;
        }
    }
}
