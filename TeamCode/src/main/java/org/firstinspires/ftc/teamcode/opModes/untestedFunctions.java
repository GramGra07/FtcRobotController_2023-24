package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class untestedFunctions extends HardwareConfig{

    public untestedFunctions(LinearOpMode opmode) {
        super(opmode);
    }

    public void antiTip(){
        double maxRoll = 10;
        double minRoll = -10;
        if (roll > maxRoll) {
            //tipped to right
            motorBackLeft.setPower(-1);
            motorFrontLeft.setPower(1);
            if (roll<maxRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        if (roll < minRoll) {
            //tipped to left
            motorBackLeft.setPower(1);
            motorFrontLeft.setPower(-1);
            if (roll>minRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        double maxPitch = 10;
        double minPitch = -10;
        if (pitch > maxPitch) {
            //tipped to front
            motorFrontRight.setPower(1);
            motorFrontLeft.setPower(1);
            telemetry.update();
            if (pitch<maxPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
        if (pitch < minPitch) {
            //tipped to back
            motorBackRight.setPower(-1);
            motorBackLeft.setPower(-1);
            if (pitch>minPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }
    }
}
