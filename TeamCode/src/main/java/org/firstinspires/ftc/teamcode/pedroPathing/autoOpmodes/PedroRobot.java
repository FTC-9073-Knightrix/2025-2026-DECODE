package org.firstinspires.ftc.teamcode.pedroPathing.autoOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PedroRobot {
    // This class holds all the mechanisms on the robot
    // that will run during pedro pathing
    public DcMotorEx intakeMotor;
    public DcMotorEx intake2Motor;

    final private double INTAKE_POWER = -1.0;
    final private double STOP_POWER = 0.0;

    // Outtake
    public DcMotorEx outtakeMotor;
    public Servo hoodServo;
    public final double midShotTargetVelocityTicks = -1175.0;
    public final double farShotTargetVelocityTicks = -1480.0;
    public double hoodPosition = 0.75;

    final private double ACCEPTABLE_VELOCITY_ERROR = 50.0;

    final double MID_SHOT_HOOD = 0.75;
    final double FAR_SHOT_HOOD = 0.45;

    // ------------------------------- Transfer Motor --------------------------------
    public DcMotor transferMotor;
    final double TRANSFER_IN_POWER = 1.0;
    final double TRANSFER_STOP_POWER = 0.0;

    final int BALLS_TO_TRANSFER = 3;

    public RevBlinkinLedDriver blinkin;

    enum TransferState {
        WAITING_FOR_SPEED,      // Waiting for flywheel to reach target speed
        TRANSFERRING,           // Transfer motor running, feeding ball
        RECOVERING              // Waiting for flywheel to recover before next ball
    }

    // Transfer state machine fields
    private TransferState transferState = TransferState.WAITING_FOR_SPEED;
    private int ballsTransferred = 0;
    private double transferTargetVelocity = 0;
    private boolean transferInitialized = false;
    private boolean transferStillRunning = false;

    // Timers for transfer state machine
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();

    final double RECOVERY_TIME = 0.25; // seconds to wait after detecting a ball
    final double MAX_TRANSFER_TIME = 1.0; // seconds per ball max
    final double MAX_TOTAL_TRANSFER_TIME = 4.5; // total time for all balls

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

        // Transfer
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Blinkin
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    // ------------------------------- Intake Methods --------------------------------
    public void runIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        intake2Motor.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(STOP_POWER);
        intake2Motor.setPower(STOP_POWER);
    }
    // ------------------------------- Outtake Methods --------------------------------
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

    private boolean velocityDroppedBelowThreshold(double targetVelocity) {
        double currentVelocity = outtakeMotor.getVelocity();

        double absTarget = Math.abs(targetVelocity);
        double absCurrent = Math.abs(currentVelocity);

        // Define a threshold in ticks that constitutes a real drop from ball impact
        final double DROP_THRESHOLD_TICKS = 110.0;

        // A drop is when current magnitude has fallen by at least DROP_THRESHOLD_TICKS below target magnitude
        return absCurrent <= (absTarget - DROP_THRESHOLD_TICKS);
    }
    // ------------------------------- Transfer Methods --------------------------------

    /**
     * Initializes the transfer process. Call this before starting to call runTransfer().
     * @param targetVelocity The target velocity the shooter should be at
     */
    public void initTransfer(double targetVelocity) {
        ballsTransferred = 0;
        transferState = TransferState.WAITING_FOR_SPEED;
        transferMotor.setPower(TRANSFER_STOP_POWER);
        transferInitialized = true;
        transferStillRunning = true;
        transferTargetVelocity = targetVelocity;
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        // Reset all timers
        transferTimer.reset();
        stateTimer.reset();
        recoveryTimer.reset();
    }

    /**
     * Runs the transfer state machine. Call this repeatedly in a loop.
     * Check transferStillRunning() to see if transfer is complete.
     */
    public void runTransfer() {
        if (!transferInitialized) {
            transferStillRunning = false;
            return;
        }

        switch (transferState) {
            case WAITING_FOR_SPEED:
                transferMotor.setPower(TRANSFER_STOP_POWER);
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                if (isShooterAtVelocity(transferTargetVelocity)) {
                    transferState = TransferState.TRANSFERRING;
                    stateTimer.reset();
                    // Start feeding and keep motor on while we wait for a drop
                    transferMotor.setPower(TRANSFER_IN_POWER);
                }
                break;

            case TRANSFERRING:
                // Keep running the transfer motor until we detect a velocity drop
                transferMotor.setPower(TRANSFER_IN_POWER);
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                if (velocityDroppedBelowThreshold(transferTargetVelocity)) {
                    // Ball impacted the flywheel
                    ballsTransferred += 1;
                    transferMotor.setPower(TRANSFER_STOP_POWER);
                    transferState = TransferState.RECOVERING;
                    recoveryTimer.reset();
                }
                break;

            case RECOVERING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                transferMotor.setPower(TRANSFER_STOP_POWER);
                if (recoveryTimer.seconds() >= RECOVERY_TIME) {
                    if (ballsTransferred < BALLS_TO_TRANSFER) {
                        transferState = TransferState.WAITING_FOR_SPEED;
                        stateTimer.reset();
                    }
                }
                break;
        }

        // Update stillRunning status
        transferStillRunning = ballsTransferred < BALLS_TO_TRANSFER;

        if (!transferStillRunning) {
            transferMotor.setPower(TRANSFER_STOP_POWER);
            transferInitialized = false;
        }
    }

    /**
     * Checks if the transfer is still running
     * @return true if still running, false if complete
     */
    public boolean transferStillRunning() {
        return transferStillRunning;
    }

    /**
     * Gets the number of balls transferred so far
     */
    public int getBallsTransferred() {
        return ballsTransferred;
    }

    /**
     * Resets the transfer state machine
     */
    public void resetTransfer() {
        transferInitialized = false;
        ballsTransferred = 0;
        transferState = TransferState.WAITING_FOR_SPEED;
        transferMotor.setPower(TRANSFER_STOP_POWER);
        transferStillRunning = false;
    }

    public void stopTransfer() {
        transferMotor.setPower(TRANSFER_STOP_POWER);
        transferInitialized = false;
        transferStillRunning = false;
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }
}
