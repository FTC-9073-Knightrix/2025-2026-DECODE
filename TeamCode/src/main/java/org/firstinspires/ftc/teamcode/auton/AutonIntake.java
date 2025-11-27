package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// UNUSED
public class AutonIntake {
    public DcMotorEx intakeMotor;

    final private double INTAKE_POWER = 1.0;
    final private double OUTTAKE_POWER = -1.0;
    final private double STOP_POWER = 0.0;

    public AutonIntake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public class RunIntake implements Action {
        private boolean isStopRequested;

        public RunIntake(boolean isStopRequested) {
            this.isStopRequested = isStopRequested;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!isStopRequested) {
                intakeMotor.setPower(INTAKE_POWER);
                return true;
            } else {
                intakeMotor.setPower(STOP_POWER);
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
            intakeMotor.setPower(STOP_POWER);
            return true;
        }
    }

    public Action stopIntake() {
        return new StopIntake();
    }
}
