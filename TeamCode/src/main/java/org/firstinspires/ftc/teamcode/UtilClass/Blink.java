package org.firstinspires.ftc.teamcode.UtilClass;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

public class Blink extends HardwareConfig {
    // to change CP aka custom team colors, or length of strip:
    // hold mode on blinkin till it is yellow
    // change it with the screw holes
    // hold both strip select and mode button till it blinks blue
    private LinearOpMode myOpMode = null;
    public static int once = 0;

    public Blink(LinearOpMode opmode) {
        super(opmode);
        myOpMode = opmode;
    }

    public static final String[] favColors = {
            "RAINBOW_RAINBOW_PALETTE",
            "BEATS_PER_MINUTE_RAINBOW_PALETTE",
            "BEATS_PER_MINUTE_PARTY_PALETTE",
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

    public static void setLights(String color, boolean freq) {
        LEDcolor = "HOT_PINK";
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
//        if (color != null) {
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(color));
//            LEDcolor = color;
//        }
//        if (color == null && !freq)
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColor()));
//        if (color == null && freq)
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColorFreq()));
    }

    public void greenRed() {
        setLights("RED", false);
        sleep(delay * 1000L);
        setLights("GREEN", false);
        isSolid = true;
    }

    public static String getColor() {
        // CP colors are chosen on the blinkin, look at top for instructions
        final int min = 0;
        final int max = favColors.length - 1;
        LEDcolor = favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
        return LEDcolor;
    }

    public static String getColorFreq() {
        // colors will be selected based on a frequency, selecting some more than others
        // 100 total pings
        // common = 66
        // uncommon = 24
        // rare = 10
        // rare+uncommon+common = 100
        final int common = 66;
        final int uncommon = 24;
        final int rare = 10;

        int min = 0;
        int max = 100;
        final int random = (int) Math.floor(Math.random() * (max - min + 1) + min);
        final String[] commonColors = {
                "HOT_PINK",
                "BEATS_PER_MINUTE_PARTY_PALETTE",
                "COLOR_WAVES_RAINBOW_PALETTE",
                "COLOR_WAVES_PARTY_PALETTE",
        };
        final String[] uncommonColors = {
                "CP2_END_TO_END_BLEND_TO_BLACK",
                "CP2_BREATH_SLOW",
                "CP1_2_END_TO_END_BLEND_1_TO_2",
                "CP1_2_END_TO_END_BLEND",
        };
        final String[] rareColors = {
                "RAINBOW_RAINBOW_PALETTE",
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",
        };
        if (random <= common) {
            // choose common
            max = commonColors.length - 1;
            min = 0;
            final int rand = (int) Math.floor(Math.random() * (max - min + 1) + min);
            LEDcolor = commonColors[rand];
        } else if (random > common && random <= common + uncommon) {
            // choose uncommon
            max = uncommonColors.length - 1;
            min = 0;
            final int rand = (int) Math.floor(Math.random() * (max - min + 1) + min);
            LEDcolor = uncommonColors[rand];
        } else if (random > uncommon && random <= common + uncommon + rare) {
            //choose rare
            max = rareColors.length - 1;
            min = 0;
            final int rand = (int) Math.floor(Math.random() * (max - min + 1) + min);
            LEDcolor = rareColors[rand];
        }
        return LEDcolor;
    }

    public static void selectLights(OpMode myOpMode) {
        if (myOpMode.gamepad2.share) {
            if (myOpMode.gamepad2.touchpad_finger_1_x < -0.5) {
                Blink.setLights("WHITE", false);
            }
            if (myOpMode.gamepad2.touchpad_finger_1_x >= -0.5 && myOpMode.gamepad2.touchpad_finger_1_x < 0) {
                Blink.setLights("GREEN", false);
            }
            if (myOpMode.gamepad2.touchpad_finger_1_x > 0 && myOpMode.gamepad2.touchpad_finger_1_x < 0.5) {
                Blink.setLights("VIOLET", false);
            }
            if (myOpMode.gamepad2.touchpad_finger_1_x >= 0.5) {
                Blink.setLights("GOLD", false);
            }
            once = 0;
        } else {
            if (once == 0) {
                Blink.setLights(LEDcolor, true);
                once = 1;
            }
        }
    }
}
