package org.firstinspires.ftc.teamcode.externalHardware.teleOpX;

import static org.firstinspires.ftc.teamcode.externalHardware.OLDHardwareConfig.countsPerInchTape;
import static org.firstinspires.ftc.teamcode.externalHardware.OLDHardwareConfig.tmPose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.externalHardware.OLDHardwareConfig;

@TeleOp(name = "maintenance", group = "Robot")
//@Disabled
public class maintenanceV2 extends LinearOpMode {
    OLDHardwareConfig robot = new OLDHardwareConfig(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {//while the op mode is active
            if (isStopRequested()) return;

            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
            if (robot.touchSensor.isPressed()) {
                robot.armUp = !robot.armUp;
                robot.greenRed();
                robot.runtime.reset();
            }
            if (robot.touchSensorClaw.isPressed()) {
                robot.clawOpen = !robot.clawOpen;
                robot.greenRed();
                robot.runtime.reset();
            }
            if (robot.touchSensorEject.isPressed()) {
                robot.greenRed();
                tmPose += 2;
                robot.tmServo.setPosition(robot.setServo(tmPose));
                robot.runtime.reset();
            }
            if (robot.touchSensorL.isPressed()) {
                robot.greenRed();
                robot.tapeOut = !robot.tapeOut;
                robot.runtime.reset();
            }
            if (robot.armUp) {
                robot.pitchEncoder(80, 1, 6, false);
                robot.green1.setState(true);
                robot.red1.setState(false);
            } else {
                robot.pitchEncoder(0, 1, 6, true);
                robot.green1.setState(false);
                robot.red1.setState(true);
            }
            if (robot.tapeOut) {
                robot.tapeEncoder((int) (countsPerInchTape * 10 * 18), 1, 6, false);//go out
                robot.green3.setState(true);
                robot.red3.setState(false);
            } else {
                robot.tapeEncoder(0, 1, 6, true);// come in
                robot.green3.setState(false);
                robot.red3.setState(true);
            }
            if (robot.clawOpen) {
                robot.openClaw();
                robot.green2.setState(false);
                robot.red2.setState(true);
            } else {
                robot.closeClaw();
                robot.green2.setState(true);
                robot.red2.setState(false);
            }
            telemetry.addData("armUp", robot.armUp);
            telemetry.addData("clawOpen", robot.clawOpen);
            telemetry.addData("color", robot.color);
            telemetry.addData("tapeOut", robot.tapeOut);
            telemetry.addData("tmPose", tmPose);
            telemetry.addData("tmEncoder", robot.tapeMeasure.getCurrentPosition());
            telemetry.update();
        }
    }
}