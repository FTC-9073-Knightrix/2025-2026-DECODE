package org.firstinspires.ftc.teamcode.teleop.main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1: DECODE TeleOp")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void loop() {
        // THIS IS THE MAIN RUN LOOP FOR THE ROBOT
        // PUT METHODS IN HERE THAT U WANT TO BE CONTINUOUSLY RUNNING
        // WHILE WE R CONTROLLING THE ROBOT
        displayTelemetry(); // this shows text on the driver hub
    }
}