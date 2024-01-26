package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Limits {
    public static double flipperMin = -0.7, flipperMax = 0.7;
    public static double liftMax = 1, liftMin = -0.7;
    public static double slideMax = 1, slideMin = -1;
    public static int autoExtension = 630;
    public static int maxExtensionTicks = 1000;
    public static int minExtensionTicks = 0;
    public static int maxRotationTicks = 100;
    public static int minRotationTicks = 0;
}
