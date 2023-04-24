package org.firstinspires.ftc.teamcode.externalHardware.autoX;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.externalHardware.autoHardware;

@Deprecated
@Autonomous(name = "minimalAutoX", group = "Robot")
//@Disabled
public class OLDminimalAuto extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(robot.getColor()));
            robot.runVu(6, true);
            double fwd = 30;
            double sdw = -10;
            resetRuntime();
            robot.encoderDrive(1, 1, 1, 0.5);
            //robot.yArmEncoder(500, 1, 2, false);
            if (robot.spot == 1) {
                robot.green1.setState(true);
                robot.red1.setState(false);
                robot.sideWaysEncoderDrive(0.5, sdw, 3);
                sleep(50);
                robot.resetEncoders();
                robot.encoderDrive(0.5, fwd, fwd, 3);
            } else if (robot.spot == 2) {
                robot.green1.setState(true);
                robot.red1.setState(false);
                robot.green2.setState(true);
                robot.red2.setState(false);
                robot.resetEncoders();
                robot.encoderDrive(0.5, fwd, fwd, 3);
            } else if (robot.spot == 3) {
                robot.green1.setState(true);
                robot.red1.setState(false);
                robot.green2.setState(true);
                robot.red2.setState(false);
                robot.green3.setState(true);
                robot.red3.setState(false);
                robot.sideWaysEncoderDrive(0.5, -sdw, 3);
                sleep(50);
                robot.resetEncoders();
                robot.encoderDrive(0.5, fwd, fwd, 3);
            } else if (robot.spot == 4) {
                robot.sideWaysEncoderDrive(0.5, -sdw - 3, 3);
                robot.green1.setState(true);
                robot.red1.setState(false);
                robot.green2.setState(true);
                robot.red2.setState(false);
                robot.resetEncoders();
                robot.encoderDrive(0.5, fwd, fwd, 3);
            }
            //robot.yArmEncoder(0, 1, 2, true);
            telemetry.update();
        }
    }
}