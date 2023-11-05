package org.firstinspires.ftc.teamcode.ggutil.sensors.voltage;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "voltageMultTest", group = "Robot")
@Disabled
public class voltageMultTest extends LinearOpMode {

    public boolean slowModeIsOn = false;
    public boolean testRunning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        //Servo clawServo = hardwareMap.Servo.get("clawServo");
        DcMotorEx sparkLong = hardwareMap.get(DcMotorEx.class, "sparkLong");
        //DigitalChannel left= (DigitalChannel) hardwareMap.get("leftLed");
        //DigitalChannel middleLed= (DigitalChannel) hardwareMap.get("middleLed");
        //DigitalChannel right= (DigitalChannel) hardwareMap.get("rightLed");
        DcMotorEx deadWheel = hardwareMap.get(DcMotorEx.class, "deadWheel");
        DcMotorEx deadWheelR = hardwareMap.get(DcMotorEx.class, "deadWheelR");
        DcMotorEx deadWheelL = hardwareMap.get(DcMotorEx.class, "deadWheelL");
        deadWheelR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        deadWheelR.setZeroPowerBehavior(BRAKE);
        deadWheelR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            deadWheelR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (testRunning) {
                deadWheelR.setPower(1);
                sleep(10 * 1000);
            } else {
                deadWheelR.setPower(0);
            }

            telemetry.addData("encoder ticks", "");
            telemetry.addData("", String.valueOf(deadWheelR.getCurrentPosition()));
            telemetry.update();
        }
    }


}