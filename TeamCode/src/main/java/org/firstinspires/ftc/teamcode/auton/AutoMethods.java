package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoMethods {
    // ------------------------------- Intake Motor --------------------------------
    public DcMotorEx intakeMotor;
    final private double INTAKE_VELOCITY = 2000;   // TODO: 11/16/25 TUNE THIS VALUE ASAP
    final private double STOP_VELOCITY = 0.0;

    // ------------------------------- Outtake Motor --------------------------------
    public DcMotorEx outtakeMotor;
    public Servo hoodServo;
    public double targetVelocityTicks = 0.0;
    static final double TICKS_PER_REV = 28;
    public double hoodPosition = 0.0;

    // ------------------------------- Transfer Motor --------------------------------
    public enum DetectedColor {  // these are the colors we detect, i might need to tune the values later
        // TODO: 11/16/25 Make it so it detects at a certain distance and color range
        GREEN,
        PURPLE,
        NONE
    }

    public DcMotor transferMotor;
    private final double TRANSFER_IN_POWER = 0.5;
    private final double TRANSFER_OUT_POWER = -0.5;
    private final double TRANSFER_STOP_POWER = 0.0;

    private DetectedColor currentColor = DetectedColor.NONE;

    // ------------------------------- Blinkin --------------------------------
    public Servo blinkin;
    private final double COLOR_INTAKING = 0.61;  // red goes BRRRR am i right andrew :)
    private final double COLOR_SHOOTING = 0.95;  /* blue as in the color
    of blood that will be falling
    from ur veins after i skin u alive my
    little goodluck charm andy!
    */

    // ------------------------------- Constructor --------------------------------
    public AutoMethods(HardwareMap hardwareMap) {
        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Outtake
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(hoodPosition);

        // Transfer
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        // Blinkin
        blinkin = hardwareMap.get(Servo.class, "blinkin");
    }

    // ------------------------------- Transfer Helpers --------------------------------
    public void setDetectedColor(DetectedColor color) {
        this.currentColor = color;
    }

    private boolean isColorValid() {
        return currentColor == DetectedColor.GREEN || currentColor == DetectedColor.PURPLE;
    }

    // ------------------------------- Intake Actions --------------------------------
    public class RunIntake implements Action {
        private boolean isStopRequested;

        public RunIntake(boolean isStopRequested) {
            this.isStopRequested = isStopRequested;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!isStopRequested) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
                blinkin.setPosition(COLOR_INTAKING);   // red for intake
                packet.put("Intake Velocity", INTAKE_VELOCITY);
                return true;
            } else {
                intakeMotor.setVelocity(STOP_VELOCITY);
                packet.put("Intake", "Stopped");
                return false;
            }
        }
    }

    public Action runIntake(boolean isStopRequested) {
        return new RunIntake(isStopRequested);
    }

    public class StopIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setVelocity(STOP_VELOCITY);
            packet.put("Intake", "Stopped");
            return false;
        }
    }

    public Action stopIntake() {
        return new StopIntake();
    }

    // ------------------------------- Outtake Actions --------------------------------
    public class OuttakeVelocityAction implements Action {
        private double velocity;

        public OuttakeVelocityAction(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeMotor.setVelocity(velocity);
            blinkin.setPosition(COLOR_SHOOTING);  // blue when shooting
            packet.put("Outtake Target Vel", velocity);
            packet.put("Outtake Actual Vel", outtakeMotor.getVelocity());
            return false;
        }
    }

    public Action setOuttakeVelocity(double velocity) {
        return new OuttakeVelocityAction(velocity);
    }

    public class StopOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeMotor.setVelocity(0);
            packet.put("Outtake", "Stopped");
            return false;
        }
    }

    public Action stopOuttake() {
        return new StopOuttake();
    }

    // ------------------------------- Hood Actions --------------------------------
    public class HoodPositionAction implements Action {
        double pos;

        public HoodPositionAction(double pos) {
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            hoodServo.setPosition(pos);
            packet.put("Hood Pos", pos);
            return false;
        }
    }

    public Action setHood(double pos) {
        return new HoodPositionAction(pos);
    }

    // ------------------------------- Transfer Action --------------------------------
    public class TransferWhenReadyAction implements Action {
        private final double requiredVelocity;
        private boolean hasTransferred = false;

        public TransferWhenReadyAction(double requiredVelocity) {
            this.requiredVelocity = requiredVelocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double currentVel = outtakeMotor.getVelocity();
            packet.put("Outtake Vel", currentVel);
            packet.put("Required Vel", requiredVelocity);
            packet.put("Color Valid", isColorValid());

            if (hasTransferred) {
                transferMotor.setPower(TRANSFER_STOP_POWER);
                packet.put("Transfer", "done");
                return false;
            }
            // this is just for testing, we dont need this in the final version i also gpted it lol (the prints btw)
            if (currentVel >= requiredVelocity && isColorValid()) {
                transferMotor.setPower(TRANSFER_IN_POWER);
                packet.put("Transfer", "in, ready");
                hasTransferred = true;
                return true;
            }
            transferMotor.setPower(TRANSFER_STOP_POWER);
            packet.put("Transfer", "waiting");
            return true;
        }
    }

    public Action transferWhenReady(double requiredVelocity) {
        return new TransferWhenReadyAction(requiredVelocity);
    }
}