package org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars;

//@Config
public class blueconeObjVars {
    public static double aspectRatio = 0.7333333333333333;
    public static double minWidth = 53.0;
    public static double minHeight = 74.0;
    public static double maxWidth = 228.0;
    public static double maxHeight = 240.0;
    public static double minArea = minWidth * minHeight;
    public static double maxArea = maxWidth * maxHeight;
    public static double tolerance = 0.2; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    public static double translationX = 1.7142857142857142;
    public static double translationY = 8.033898305084746;
}