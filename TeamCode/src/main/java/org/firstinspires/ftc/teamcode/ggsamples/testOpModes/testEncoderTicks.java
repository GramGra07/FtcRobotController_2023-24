package org.firstinspires.ftc.teamcode.ggsamples.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;


@TeleOp
public class testEncoderTicks extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {//if opmode is started
        robot.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            telemetry.addData("enc1", HardwareConfig.enc1.getCurrentPosition());
            telemetry.update();
        }
    }
}