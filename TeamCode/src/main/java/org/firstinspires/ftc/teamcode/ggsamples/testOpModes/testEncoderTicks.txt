package org.firstinspires.ftc.teamcode.ggsamples.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;


@TeleOp(name = "testEncoderTicks", group = "Robot")
@Disabled//disabling the opmode
public class testEncoderTicks extends LinearOpMode {//declaring the class
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {//if opmode is started
        robot.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            telemetry.addData("fl", robot.motorFrontLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}