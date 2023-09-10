package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

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
}
