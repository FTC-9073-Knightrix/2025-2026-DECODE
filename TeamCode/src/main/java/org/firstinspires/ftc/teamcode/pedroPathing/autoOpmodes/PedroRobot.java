package org.firstinspires.ftc.teamcode.pedroPathing.autoOpmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PedroRobot {
    // Intake
    public DcMotorEx intakeMotor;
    public DcMotorEx intake2Motor;

    final private double INTAKE_POWER = -1.0;
    final private double STOP_POWER = 0.0;

    // Gate Servo
    public Servo gateServo;
    public final double GATE_OPEN_POSITION = 0.55;
    public final double GATE_CLOSED_POSITION = 0.42;

    // Turret Motor
    public DcMotorEx turretMotor;
    public double closeZoneTurretAngle = 0.0;
    public double farZoneTurretAngle = 90.0;

    // Outtake
    public DcMotorEx outtakeMotor;
    public Servo hoodServo;
    public final double midShotTargetVelocityTicks = -1175.0;
    public final double farShotTargetVelocityTicks = -1480.0;
    public double hoodPosition = 0.75;

    final private double ACCEPTABLE_VELOCITY_ERROR = 50.0;

    final double MID_SHOT_HOOD = 0.8;
    final double FAR_SHOT_HOOD = 0.3;

    public RevBlinkinLedDriver blinkin;

    public PedroRobot(HardwareMap hardwareMap) {
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
        final double kD = 0.01;
        final double kF = 1.0;

        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        hoodServo.setPosition(hoodPosition);

        // Gate Servo
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED_POSITION);

        // Turret Motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Blinkin
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    // Intake
    public void runIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        intake2Motor.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(STOP_POWER);
        intake2Motor.setPower(STOP_POWER);
    }

    // Outtake
    public void spinShooterToVelocity(double targetVelocityTicks) {
        outtakeMotor.setVelocity(targetVelocityTicks);
    }

    public void spinShooterForMidShot() {
        hoodServo.setPosition(MID_SHOT_HOOD);
        spinShooterToVelocity(midShotTargetVelocityTicks);
    }

    public void spinShooterForFarShot() {
        hoodServo.setPosition(FAR_SHOT_HOOD);
        spinShooterToVelocity(farShotTargetVelocityTicks);
    }

    public void stopShooter() {
        outtakeMotor.setPower(0.0);
    }

    public boolean isShooterAtVelocity(double targetVelocityTicks) {
        double currentVelocity = outtakeMotor.getVelocity();
        return Math.abs(currentVelocity - targetVelocityTicks) <= ACCEPTABLE_VELOCITY_ERROR;
    }

    // Gate Servo
    public void openGate() {
        gateServo.setPosition(GATE_OPEN_POSITION);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED_POSITION);
    }
}
