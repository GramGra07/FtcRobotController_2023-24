package org.firstinspires.ftc.teamcode.externalHardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class untestedFunctions extends HardwareConfig{

    public untestedFunctions(LinearOpMode opmode) {
        super(opmode);
    }
    public String getColorFrequency(){
        double commonP = 0.2;
        double uncommonP = 0.1;
        double rareP = 0.057;
        double pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine, pTen, pEleven;
        pOne = pTwo = pThree = pFour = pFive = pNine = pEleven = rareP;//7 * rareP  = 0.399
        pSix = pTen = uncommonP;//2 * uncommonP = 0.2
        pSeven = pEight = commonP;//2 * commonP = 0.4

        double[] p = {pOne, pTwo, pThree, pFour, pFive, pSix, pSeven, pEight, pNine, pTen, pEleven};
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",//1
                "RAINBOW_PARTY_PALETTE",//2
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",//3
                "BEATS_PER_MINUTE_PARTY_PALETTE",//4
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
            LEDcolor = favColors[0];
            return favColors[0];
        } else if (ran < p[2] && ran > p[1]) {
            LEDcolor = favColors[1];
            return favColors[1];
        } else if (ran < p[3] && ran > p[2]) {
            LEDcolor = favColors[2];
            return favColors[2];
        } else if (ran < p[4] && ran > p[3]) {
            LEDcolor = favColors[3];
            return favColors[3];
        } else if (ran < p[5] && ran > p[4]) {
            LEDcolor = favColors[4];
            return favColors[4];
        } else if (ran < p[6] && ran > p[5]) {
            LEDcolor = favColors[5];
            return favColors[5];
        } else if (ran < p[7] && ran > p[6]) {
            LEDcolor = favColors[6];
            return favColors[6];
        } else if (ran < p[8] && ran > p[7]) {
            LEDcolor = favColors[7];
            return favColors[7];
        } else if (ran < p[9] && ran > p[8]) {
            LEDcolor = favColors[8];
            return favColors[8];
        } else if (ran < p[10] && ran > p[9]) {
            LEDcolor = favColors[9];
            return favColors[9];
        } else {
            LEDcolor = favColors[10];
            return favColors[10];
        }
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
