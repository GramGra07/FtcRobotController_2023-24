package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.opencv.core.Rect;

public class DetectionResult {
    private static Rect boundingBox;
    private String classLabel;
    private float confidenceScore;

    public DetectionResult(Rect boundingBox, String classLabel, float confidenceScore) {
        this.boundingBox = boundingBox;
        this.classLabel = classLabel;
        this.confidenceScore = confidenceScore;
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
}
