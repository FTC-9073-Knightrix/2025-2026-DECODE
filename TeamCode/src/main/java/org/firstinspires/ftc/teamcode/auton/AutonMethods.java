package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonMethods extends AutonBase {
    // ------------------------------- Intake Motor --------------------------------
    public DcMotorEx intakeMotor;
    public DcMotorEx intake2Motor;
    final private double INTAKE_POWER = -1.0;
    final private double STOP_POWER = 0.0;

    // ------------------------------- Outtake Motor --------------------------------
    public DcMotorEx outtakeMotor;
    public Servo hoodServo;
    public double midShotTargetVelocityTicks = -1150.0;
    static final double TICKS_PER_REV = 28;
    public double hoodPosition = 0.72;

    final private double ACCEPTABLE_VELOCITY_ERROR = 75.0;

    // ------------------------------- Transfer Motor --------------------------------
    public DcMotor transferMotor;
    public DistanceSensor transferDistanceSensor;
    final double TRANSFER_IN_POWER = 0.6;
    final double TRANSFER_OUT_POWER = -1.0;
    final double TRANSFER_STOP_POWER = 0.0;
    final double DISTANCE_TO_DETECT_BALL_CM = 2.0; // in cm


    // ------------------------------- Blinkin --------------------------------
    public RevBlinkinLedDriver blinkin;
    private final double COLOR_INTAKING = 0.61;  // red goes BRRRR am i right andrew :)
    private final double COLOR_SHOOTING = 0.95;  /* blue as in the color
    */

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public class AutonActions {

        // ------------------------------- Constructor --------------------------------
        public AutonActions(HardwareMap hardwareMap) {
            // Intake
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
            intake2Motor = hardwareMap.get(DcMotorEx.class, "intake2");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Outtake
            outtakeMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            hoodServo = hardwareMap.get(Servo.class, "hoodServo");

            outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double kP = 30;
            double kI = 0.9;
            double kD = 0.0;
            double kF = 0.7;

            outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            hoodServo.setPosition(hoodPosition);

            // Transfer
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferDistanceSensor = hardwareMap.get(DistanceSensor.class, "transferDistanceSensor");

            // Blinkin
            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        }

        // ------------------------------- Intake Actions --------------------------------
        public class RunIntake implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(INTAKE_POWER);
                    intake2Motor.setPower(INTAKE_POWER);
                    initialized = true;
                }
                return false;
            }
        }

        public Action runIntake() {
            return new RunIntake();
        }

        public class StopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(STOP_POWER);
                intake2Motor.setPower(STOP_POWER);
                packet.put("Intake", "Stopped");
                return false;
            }
        }

        public Action stopIntake() {
            return new StopIntake();
        }

        // ------------------------------- Outtake Actions --------------------------------
        public class SpinShooterToMidShotVelocity implements Action {
            boolean initialized = false;
            double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtakeMotor.setVelocity(midShotTargetVelocityTicks);
                    startTime = getRuntime();

                    initialized = true;
                }

                packet.put("Outtake Target Vel", midShotTargetVelocityTicks);
                packet.put("Outtake Actual Vel", outtakeMotor.getVelocity());
                return (Math.abs(outtakeMotor.getVelocity() - midShotTargetVelocityTicks) > ACCEPTABLE_VELOCITY_ERROR) || (getRuntime() - startTime < 2);
            }
        }

        public Action spinShooterToMidShotVelocity() {
            return new SpinShooterToMidShotVelocity();
        }

        public class OuttakeVelocityAction implements Action {
            private double velocity;

            public OuttakeVelocityAction(double velocity) {
                this.velocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeMotor.setVelocity(velocity);
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

        public class StallOuttake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeMotor.setVelocity(-400);
                packet.put("Outtake", "Stalled");
                return false;
            }
        }
        public Action stallOuttake() {
            return new StallOuttake();
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

        public class SetHoodToMidShot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hoodServo.setPosition(0.9);
                packet.put("Hood Pos", 0.9);
                return false;
            }
        }

        public Action setHoodToMidShot() {
            return new SetHoodToMidShot();
        }

        // ------------------------------- Transfer Action --------------------------------
        public class RunTransferUntilBallDetected implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStopRequested()) {
                    if (!initialized) {
                        transferMotor.setPower(TRANSFER_IN_POWER);
                        initialized = true;
                    }
                    double distance = transferDistanceSensor.getDistance(DistanceUnit.CM); // in cm
                    if (Double.isNaN(distance) || distance <= 0) {
                        distance = Double.POSITIVE_INFINITY;
                    }
                    if (distance < DISTANCE_TO_DETECT_BALL_CM) {
                        transferMotor.setPower(TRANSFER_STOP_POWER);
                        packet.put("Transfer", "Ball Detected, Stopped");
                        return false;
                    } else {
                        return true;
                    }
                } else {
                    transferMotor.setPower(TRANSFER_STOP_POWER);
                    packet.put("Transfer", "Stopped");
                    return false;
                }
            }
        }

        public Action runTransferUntilBallDetected() {
            return new RunTransferUntilBallDetected();
        }

        public class RunTransfer implements Action {
            boolean initialized = false;
            double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStopRequested()) {
                    if (!initialized) {
                        transferMotor.setPower(TRANSFER_IN_POWER);
                        startTime = getRuntime();
                        initialized = true;
                    }
                    return getRuntime() - startTime < 4;
                } else {
                    transferMotor.setPower(TRANSFER_STOP_POWER);
                    packet.put("Transfer", "Stopped");
                    return false;
                }
            }
        }

        public Action runTransfer() {
            return new RunTransfer();
        }

        public class StopTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transferMotor.setPower(TRANSFER_STOP_POWER);
                packet.put("Transfer", "Stopped");
                return false;
            }
        }
        public Action stopTransfer() {
            return new StopTransfer();
        }
    }
}