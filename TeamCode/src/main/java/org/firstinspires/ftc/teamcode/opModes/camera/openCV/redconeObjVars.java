package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import com.acmerobotics.dashboard.config.Config;

@Config
public class redconeObjVars {
    public static double aspectRatio = 0.7033898305084746;
    public static double minWidth = 53.0;
    public static double minHeight = 78.0;
    public static double maxWidth = 119.0;
    public static double maxHeight = 199.0;
    public static double minArea = minWidth * minHeight;
    public static double maxArea = maxWidth * maxHeight;
    public static double tolerance = 0.2; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    public static double translationX = 95.0;
    public static double translationY = 5.647058823529412;
}