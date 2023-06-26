package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public class DetectionResult {
    private static Rect boundingBox;
    private String classLabel;
    private float confidenceScore;

    public DetectionResult(Rect boundingBox, String classLabel, float confidenceScore) {
        this.boundingBox = boundingBox;
        this.classLabel = classLabel;
        this.confidenceScore = confidenceScore;
    }
    static void convertMatToFloatArray(Mat mat, float[][][] array) {
        // Convert the input OpenCV Mat to a float array for TensorFlow Lite model input
        int channels = mat.channels();
        int width = mat.width();
        int height = mat.height();

        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                double[] pixel = mat.get(row, col);
                for (int channel = 0; channel < channels; channel++) {
                    array[row][col][channel] = (float) pixel[channel];
                }
            }
        }
    }
    public static Rect getBoundingBox() {
        return boundingBox;
    }

    public String getClassLabel() {
        return classLabel;
    }

    public float getConfidenceScore() {
        return confidenceScore;
    }
    // ...

    public static List<DetectionResult> postprocessResults(float[][] outputArray) {
        List<DetectionResult> detectionResults = new ArrayList<>();
        for (float[] detection : outputArray) {
            float x = detection[0];
            float y = detection[1];
            float w = detection[2];
            float h = detection[3];
            Rect boundingBox = new Rect((int) x, (int) y, (int) w, (int) h); // Replace with actual values
            String classLabel = "object"; // Replace with actual class label
            float confidenceScore = 0.9f; // Replace with actual confidence score
            DetectionResult result = new DetectionResult(boundingBox, classLabel, confidenceScore);
            detectionResults.add(result);
        }
        return detectionResults;
    }
}
