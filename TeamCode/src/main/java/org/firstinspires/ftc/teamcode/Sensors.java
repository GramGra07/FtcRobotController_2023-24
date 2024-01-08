package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.UtilClass.ServoUtil.closeClaw;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

public class Sensors {
    public static boolean limitSwitchState = false;
    public static final double POTENTIOMETER_MAX = 270;
    public static final double POTENTIOMETER_MIN = 0;

    public static boolean lowVoltage = false;
    public static final double minimumVoltage = 11.5;
    public static double currentVoltage;


    public static double getPotentVal(AnalogInput potentiometer) {
        return Range.clip(POTENTIOMETER_MAX / 3.3 * potentiometer.getVoltage(), POTENTIOMETER_MIN, POTENTIOMETER_MAX);
    }

    public static void driveByPotentVal(int target, AnalogInput potent, DcMotor motor) {
        double dif = target - Sensors.getPotentVal(potent);
        double range = 1;
        // turn motor until dif > 1 or close
        while (Math.abs(dif) > range) {
            double sign = (dif / Math.abs(dif));
            dif = target - Sensors.getPotentVal(potent);
            motor.setPower(sign * 0.5);
        }
        motor.setPower(0);
    }

    public static double calculatePowerByPotent(int target, AnalogInput potent, DcMotor motor) {
        double dif = target - Sensors.getPotentVal(potent);
        double range = 1;
        // turn motor until dif > 1 or close
        if (Math.abs(dif) > range) {
            double sign = -(dif / Math.abs(dif));
            dif = target - Sensors.getPotentVal(potent);
            return sign * 0.5;
        } else {
            return 0;
        }
    }

    public static boolean getLimitSwitch(DigitalChannel limitSwitch) {
        limitSwitchState = !limitSwitch.getState();
        return limitSwitchState;
    }

    public static void getBatteryVoltage(VoltageSensor vSensor) { //VoltageSensor sensor
        double result = Double.POSITIVE_INFINITY;
        double voltage = vSensor.getVoltage();
        if (voltage > 0) {
            result = Math.min(result, voltage);
        }
        lowVoltage = result <= minimumVoltage;
        currentVoltage = result;
    }

    public static void ledIND(DigitalChannel green, DigitalChannel red, boolean greenOn) {
        green.setState(greenOn);
        red.setState(!greenOn);
    }

    public static boolean getTouchSensor(TouchSensor touchSensor) {
        return touchSensor.isPressed();
    }

    public static double[] loadColorDistSensor(NormalizedColorSensor colorSensor) {
        double distance = 0;
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }
        return new double[]{colors.red, colors.green, colors.blue, distance / 2};
    }

    public static double[] getColors(NormalizedColorSensor colorSensor) {
        return new double[]{loadColorDistSensor(colorSensor)[0], loadColorDistSensor(colorSensor)[1], loadColorDistSensor(colorSensor)[2]};
    }

    public static double getDistance(NormalizedColorSensor colorSensor) {
        return loadColorDistSensor(colorSensor)[3];
    }

    public static void operateClawByDist(NormalizedColorSensor colorSensor) {
        Servo claw = null;
        if (colorSensor == HardwareConfig.colorSensorC1) {
            claw = HardwareConfig.claw1;
        } else if (colorSensor == HardwareConfig.colorSensorC2) {
            claw = HardwareConfig.claw2;
        }
        double distance = getDistance(colorSensor);
        if (claw != null) {
            if (distance < 2) {
                closeClaw(claw);
            } else {
                closeClaw(claw);
            }
        }
    }
}
