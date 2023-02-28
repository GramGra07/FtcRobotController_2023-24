package org.firstinspires.ftc.teamcode.externalHardware.autoX;

import static org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;
import org.firstinspires.ftc.teamcode.externalHardware.autoHardware;
@Autonomous(name = "advAutoRX", group = "Robot")
@Disabled
public class advAutoRX extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active
            robot.score1();
            double targetX = 2.1;
            double targetY2 = 3.55;
            double targetY1 = 3.55;//lined up with cones
            robot.resetEncoders();
            robot.simpleGoSpotRight(targetX, targetY2, 3.5, targetY1, robot.ovrPower, true, midPoleVal + 500,
                    true, false, 0, 1, 1, true);
            // Branch 2 place first cone
            robot.correctToCones();
            double speed = 0.4;
            robot.motorBackLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);
            robot.motorFrontLeft.setPower(speed);
            robot.motorFrontRight.setPower(speed);
            robot.yArmEncoder(fiveTallConeVal + 500, 1, 2, true);
            robot.openClaw();
            robot.yArmEncoder(fiveTallConeVal, 1.0, 0.5, true);
            sleep(500);
            robot.closeClaw();
            speed = 0;
            robot.motorBackLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);
            robot.motorFrontLeft.setPower(speed);
            robot.motorFrontRight.setPower(speed);
            robot.yArmEncoder(midPoleVal + 1000, 1, 2, false);//clear gap
            //vars
            //branch 3 get to stack
            robot.resetEncoders();
            robot.simpleGoSpotRight(ovrCurrX, ovrCurrY, targetX, targetY2, robot.ovrPower, true, topPoleVal,
                    false, false, 0, 2, 4, false);
            robot.encoderDrive(1, 2, 2, 0.5);
            robot.openClaw();
            robot.resetEncoders();
            robot.simpleGoSpotRight(ovrCurrX, ovrCurrY, targetX, targetY1, robot.ovrPower, true, midPoleVal + 500,
                    true, false, 0, 0.2, 1, true);
            robot.closeClaw();
            //2,3
            double stackDist = 19;
            robot.yArmEncoder(0, 1, 2, true);
            sleep(50);
            if (robot.spot == 3) {
                robot.encoderDrive(1, stackDist, stackDist, 3);//opposite of 3 lines higher
                //3,3
            }
            //should already be here at spot 2
            if (robot.spot == 2) {
                //2,3
            }
            if (robot.spot == 1) {
                robot.encoderDrive(1, -15, -15, 3);
                //1,3
            }
            telemetry.update();
        }
    }
}