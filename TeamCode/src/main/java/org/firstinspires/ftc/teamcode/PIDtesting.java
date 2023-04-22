package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDtesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the PID controller
        PIDController pidController = new PIDController(
                0.1, // Proportional gain
                0.01, // Integral gain
                0.05 // Derivative gain
        );

        // Initialize the motor
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setTargetPosition(1000);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive() && motor.getCurrentPosition() < motor.getTargetPosition()) {
            // Calculate the output of the PID controller
            double output = pidController.calculate(motor.getCurrentPosition(), motor.getTargetPosition());
            // Set the power of the motor based on the output of the PID controller
            motor.setPower(output);
            // Update the elapsed time
            elapsedTime.reset();
        }
    }
}
