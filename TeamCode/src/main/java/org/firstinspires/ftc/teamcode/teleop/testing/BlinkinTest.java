package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class BlinkinTest extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.x){
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                blinkinLedDriver.setPattern(pattern);
            }
            else if(gamepad1.y){
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
            }
            else if (gamepad1.b){
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(pattern);
            }
            else if (gamepad1.a){
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }
            else{
                pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
                blinkinLedDriver.setPattern(pattern);
            }


        }
    }
}