package org.firstinspires.ftc.teamcode.ggsamples.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;


@TeleOp
public class testEncoderTicks extends LinearOpMode {
    DcMotor enc1;
    @Override
    public void runOpMode() {//if opmode is started

        enc1 = hardwareMap.get(DcMotor.class, "enc1");
        enc1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {//while the op mode is active
            telemetry.addData("enc1", enc1.getCurrentPosition());
            telemetry.update();
        }
    }
}