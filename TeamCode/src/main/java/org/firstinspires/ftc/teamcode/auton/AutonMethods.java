package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.robotSubsystems.vision.AprilTagEnums;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutonMethods extends AutonBase {
    // ------------------------------- Intake Motor --------------------------------
    public DcMotorEx intakeMotor;
    public DcMotorEx intake2Motor;
    final private double INTAKE_POWER = -1.0;
    final private double STOP_POWER = 0.0;

    // ------------------------------- Outtake --------------------------------
    public DcMotorEx outtakeMotor;
    public Servo hoodServo;
    private final double midShotTargetVelocityTicks = -1200.0;
    private final double farShotTargetVelocityTicks = -1490.0;
    static final double TICKS_PER_REV = 28;
    public double hoodPosition = 0.69;

    final private double ACCEPTABLE_VELOCITY_ERROR = 40.0;

    final double MID_SHOT_HOOD = 0.7;
    final double FAR_SHOT_HOOD = 0.50;

    // ------------------------------- Transfer Motor --------------------------------
    public DcMotor transferMotor;
    public DistanceSensor transferDistanceSensor;
    final double TRANSFER_IN_POWER = 1.0;
    final double TRANSFER_STOP_POWER = 0.0;
    final double DISTANCE_TO_DETECT_BALL_CM = 2.0; // in cm

    final int BALLS_TO_TRANSFER = 3;


    // ------------------------------- Blinkin --------------------------------
    public RevBlinkinLedDriver blinkin;
    // ------------------------------- Vision --------------------------------
    public WebcamName webCam;
    public String motif = "PPG";
    VisionPortal visionPortal;

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().build();
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

            final double kP = 30;
            final double kI = 0.9;
            final double kD = 0.0;
            final double kF = 0.7;

            outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            hoodServo.setPosition(hoodPosition);

            // Transfer
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferDistanceSensor = hardwareMap.get(DistanceSensor.class, "transferDistanceSensor");

            // Blinkin
            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

            // Vision
            webCam = hardwareMap.get(WebcamName.class, "Webcam 1");
            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(webCam)
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .build();
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
        public class SpinShooterToVelocity implements Action {
            boolean initialized = false;
            double targetVelocityTicks;
            double startTime;

            public SpinShooterToVelocity(double targetVelocityTicks) {
                this.targetVelocityTicks = targetVelocityTicks;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtakeMotor.setVelocity(targetVelocityTicks);
                    startTime = getRuntime();

                    initialized = true;
                }

                return (Math.abs(outtakeMotor.getVelocity() - targetVelocityTicks) > ACCEPTABLE_VELOCITY_ERROR) || (getRuntime() - startTime < 2);
            }
        }
        public Action spinShooterToMidShotVelocity() {
            return new SpinShooterToVelocity(midShotTargetVelocityTicks);
        }

        public Action spinShooterToFarShotVelocity() {
            return new SpinShooterToVelocity(farShotTargetVelocityTicks);
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
        public class SetHood implements Action {
            double targetPos;

            public SetHood(double targetPos) {
                this.targetPos = targetPos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hoodServo.setPosition(targetPos);
                return false;
            }
        }

        public Action setHoodToMidShot() {
            return new SetHood(MID_SHOT_HOOD);
        }
        public Action setHoodToFarShot() {
            return new SetHood(FAR_SHOT_HOOD);
        }

        // ------------------------------- Transfer Action --------------------------------
        public class RunTransferUntilBallDetected implements Action {
            boolean initialized = false;
            int detectionCount = 0;
            final int REQUIRED_DETECTIONS = 2;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStopRequested()) {
                    if (!initialized) {
                        transferMotor.setPower(TRANSFER_IN_POWER);
                        initialized = true;
                        detectionCount = 0;
                    }
                    double distance = transferDistanceSensor.getDistance(DistanceUnit.CM); // in cm

                    if (distance < DISTANCE_TO_DETECT_BALL_CM) {
                        detectionCount++;
                        if (detectionCount >= REQUIRED_DETECTIONS) {
                            transferMotor.setPower(TRANSFER_STOP_POWER);
                            packet.put("Transfer", "Ball Detected, Stopped");
                            initialized = false;
                            detectionCount = 0;
                            return false;
                        } else {
                            // wait one more cycle to confirm
                            return true;
                        }
                    } else {
                        detectionCount = 0;
                        return true;
                    }
                } else {
                    transferMotor.setPower(TRANSFER_STOP_POWER);
                    packet.put("Transfer", "Stopped");
                    initialized = false;
                    detectionCount = 0;
                    return false;
                }
            }
        }

        public Action runTransferUntilBallDetected() {
            return new RunTransferUntilBallDetected();
        }

        public class RunTransfer implements Action {
            boolean initialized = false;
            boolean wentToShootingSpeed = false;
            int ballsTransferred = 0;
            double targetVelocity;
            double startTime;

            public RunTransfer(double targetVelocity) {
                this.targetVelocity = targetVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStopRequested()) {
                    if (!initialized) {
                        transferMotor.setPower(TRANSFER_IN_POWER);
                        ballsTransferred = 0;
                        startTime = getRuntime();
                        initialized = true;
                    }

                    if (isAtShootingSpeed(targetVelocity)) {
                        transferMotor.setPower(TRANSFER_IN_POWER);
                        if (!wentToShootingSpeed) {
                            wentToShootingSpeed = true;
                        }
                    }
                    else {
                        transferMotor.setPower(TRANSFER_STOP_POWER);

                        if (wentToShootingSpeed && velocityDropped(targetVelocity)) {
                            ballsTransferred += 1;
                            wentToShootingSpeed = false;
                        }
                    }

                    return ballsTransferred < BALLS_TO_TRANSFER && (getRuntime() - startTime < 2.25);
                } else {
                    transferMotor.setPower(TRANSFER_STOP_POWER);
                    packet.put("Transfer", "Stopped");
                    return false;
                }
            }
        }

        public Action runTransferToMidShot() {
            return new RunTransfer(midShotTargetVelocityTicks);
        }
        public Action runTransferToFarShot() {
            return new RunTransfer(farShotTargetVelocityTicks);
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

        // ------------------------------- Vision Actions --------------------------------
        public class ScanMotif implements Action {
            boolean initialized = false;
            boolean scanned;
            double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    scanned = false;
                    initialized = true;
                    startTime = getRuntime();
                }
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag;
                    for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
                        if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                                || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                                || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                            tag = detectedTag;
                            scanned = true;
                            if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                                motif = AprilTagEnums.OBELISK_TAG_21.getDescription();
                            } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                                motif = AprilTagEnums.OBELISK_TAG_22.getDescription();
                            } else {
                                motif = AprilTagEnums.OBELISK_TAG_23.getDescription();
                            }
                            break;
                        }
                    }
                }
                // return true if not scanned and time elapsed is less than 0.4 seconds
                return !scanned && (getRuntime() - startTime < 0.25);
            }
        }

        public Action scanMotif() {
            return new ScanMotif();
        }

        // ------------------------------- Helper Methods --------------------------------
        private boolean isAtShootingSpeed(double targetVelocity) {
            double currentVelocity = outtakeMotor.getVelocity();
            double velocityError = Math.abs(targetVelocity - currentVelocity);

            return velocityError <= ACCEPTABLE_VELOCITY_ERROR;
        }

        private boolean velocityDropped(double targetVelocity) {
            double currentVelocity = outtakeMotor.getVelocity();
            double velocityError = targetVelocity - currentVelocity;

            // A NEGATIVE NUMBER BC VELOCITY TICKS ARE NEGATIVE
            return velocityError < -80;
        }
    }
}