package org.firstinspires.ftc.teamcode.externalHardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;

public class MathFunctions extends HardwareConfig {

    public MathFunctions(LinearOpMode opmode) {
        super(opmode);
    }

    public static void antiTip() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//get and initialize the IMU
        double roll = angles.secondAngle;

        double maxRoll = 10;
        double minRoll = -10;
        while (roll > maxRoll) {
            //tipped to right
            motorBackLeft.setPower(-1);
            motorFrontLeft.setPower(1);
            if (roll<maxRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                break;
            }
        }
        while (roll < minRoll) {
            //tipped to left
            motorBackLeft.setPower(1);
            motorFrontLeft.setPower(-1);
            if (roll>minRoll) {
                //not tipped
                motorBackLeft.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                break;
            }
        }
        double pitch = -(angles.thirdAngle-180);
        double maxPitch = 10;
        double minPitch = -10;
        while (pitch > maxPitch) {
            //tipped to front
            motorFrontRight.setPower(1);
            motorFrontLeft.setPower(1);
            telemetry.addData("here", "here");
            telemetry.update();
            if (pitch<maxPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                break;
            }
        }
        while (pitch < minPitch) {
            //tipped to back
            motorBackRight.setPower(-1);
            motorBackLeft.setPower(-1);
            if (pitch>minPitch) {
                //not tipped
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                break;
            }
        }

    }

    public static int getAverage(List val) {
        int sum = 0;
        for (int i = 0; i < val.size(); i++) {
            sum += (int) val.get(i);
        }
        return sum / val.size();
    }

    public static void setOvr(double x, double y) {
        ovrCurrX = x;
        ovrCurrY = y;
    }
    public static boolean inBetween(double in, double max, double min) {
        return in < max && in > min;
    }
}
