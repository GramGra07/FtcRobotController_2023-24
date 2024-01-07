package org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars;

//@Config
public class redpropObjVars {
    public static double aspectRatio = 1;
    public static double minWidth = 109.0;
    public static double minHeight = 74.0;
    public static double maxWidth = 155.0;
    public static double maxHeight = 153.0;
    public static double minArea = minWidth * minHeight;
    public static double maxArea = maxWidth * maxHeight;
    public static double tolerance = 0.2; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    public static double translationX = 0.06896551724137931;
    public static double translationY = 0.15;
}