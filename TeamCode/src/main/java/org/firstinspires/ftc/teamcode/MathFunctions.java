package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;

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
        if (roll > maxRoll) {
            //tipped to right
            sideWaysEncoderDrive(1, -Math.toRadians(roll), 1);
        } else if (roll < minRoll) {
            //tipped to left
            sideWaysEncoderDrive(1, Math.toRadians(roll), 1);
        }
        double pitch = angles.thirdAngle + 180;
        double maxPitch = 10;
        double minPitch = -10;
        if (pitch > maxPitch) {
            //tipped to front
            encoderDrive(1, Math.toRadians(pitch), Math.toRadians(pitch), 1);
        } else if (pitch < minPitch) {
            //tipped to back
            encoderDrive(1, -Math.toRadians(pitch), -Math.toRadians(pitch), 1);
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
}
