package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class untestedFunctions extends HardwareConfig{

    public untestedFunctions(LinearOpMode opmode) {
        super(opmode);
    }
    public String getColorFrequencyTest() {
        double commonP = 0.2;
        double uncommonP = 0.1;
        double rareP = 0.057;
        double pOne;
        double pTwo;
        double pThree;
        double pFour;
        double pFive;
        double pSix;
        double pSeven;
        double pEight;
        double pNine;
        double pTen;
        double pEleven;
        pOne = pTwo = pThree = pFour = pFive = pNine = pEleven = rareP;//7 * rareP  = 0.399
        pSix = pTen = uncommonP;//2 * uncommonP = 0.2
        pSeven = pEight = commonP;//2 * commonP = 0.4

        double[] p = {pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine, pTen, pEleven};
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",//1
                "RAINBOW_PARTY_PALETTE",//2
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",//3
                "BEATS_PER_MINUTE_PARTY_PALETTE",//4
                //"FIRE_MEDIUM",
                "COLOR_WAVES_RAINBOW_PALETTE",
                "COLOR_WAVES_PARTY_PALETTE",
                "CP2_END_TO_END_BLEND_TO_BLACK",
                "CP2_BREATH_SLOW",
                "CP1_2_END_TO_END_BLEND_1_TO_2",
                "CP1_2_END_TO_END_BLEND",
                "HOT_PINK",
                "GOLD",
                "VIOLET"
        };
        //final int min = 0;
        //final int max = favColors.length - 1;
        //double ran = (int) Math.floor(Math.random() * (max - min + 1) + min);
        int min = 0;
        int max = 100;
        double ran = (int) Math.floor(Math.random() * (max - min + 1) + min);
        if (ran < p[1] && ran > 0) {
            return favColors[0];
        } else if (ran < p[2] && ran > p[1]) {
            return favColors[1];
        } else if (ran < p[3] && ran > p[2]) {
            return favColors[2];
        } else if (ran < p[4] && ran > p[3]) {
            return favColors[3];
        } else if (ran < p[5] && ran > p[4]) {
            return favColors[4];
        } else if (ran < p[6] && ran > p[5]) {
            return favColors[5];
        } else if (ran < p[7] && ran > p[6]) {
            return favColors[6];
        } else if (ran < p[8] && ran > p[7]) {
            return favColors[7];
        } else if (ran < p[9] && ran > p[8]) {
            return favColors[8];
        } else if (ran < p[10] && ran > p[9]) {
            return favColors[9];
        } else {
            return favColors[10];
        }
    }
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        if (result <=12) {//or minimum voltage
            //go red
            //override telemetry
        }
        return result;
    }

}
