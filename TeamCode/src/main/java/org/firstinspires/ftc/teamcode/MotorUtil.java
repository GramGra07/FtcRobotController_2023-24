package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorUtil {
    public void resetEncoder(@NonNull DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runWithoutEncoder(@NonNull DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runWithEncoder(@NonNull DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDirectionR(@NonNull DcMotor motor){
        motor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setDirectionF(@NonNull DcMotor motor){
        motor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void zeroPowerBrake(@NonNull DcMotor motor){
        motor.setZeroPowerBehavior(BRAKE);
    }
    public double getCurrentPosition(@NonNull DcMotor motor){
        return motor.getCurrentPosition();
    }
    public void setTargetPosition(@NonNull DcMotor motor, int pose){
        motor.setTargetPosition(pose);
    }
    public void runToPose(@NonNull DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
