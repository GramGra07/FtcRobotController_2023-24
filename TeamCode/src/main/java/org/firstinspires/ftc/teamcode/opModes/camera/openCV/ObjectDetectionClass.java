package org.firstinspires.ftc.teamcode.opModes.camera.openCV;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionClass {
    private class OBJDetect extends OpenCvPipeline {
        int numClasses = 2;
        private Interpreter tflite;

        public OBJDetect(String modelPath) {
            try {
                tflite = new Interpreter(new File(modelPath));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        @Override
        public Mat processFrame(Mat frame) {

            // Preprocess input frame
            Mat preprocessedFrame = preprocessFrame(frame);

            // Create input and output arrays for TensorFlow Lite model
            float[][][] inputArray = new float[frame.height()][frame.width()][frame.channels()];
            float[][] outputArray = new float[1][numClasses];

            // Convert preprocessed frame to input array for TensorFlow Lite model
            DetectionResult.convertMatToFloatArray(preprocessedFrame, inputArray);

            // Run inference
            tflite.run(inputArray, outputArray);

            // Post-process results
            List<DetectionResult> detectionResults = DetectionResult.postprocessResults(outputArray);

            // Visualize results
            visualizeResults(frame, detectionResults);
            return frame;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat preprocessedFrame = new Mat();
            Imgproc.cvtColor(frame, preprocessedFrame, Imgproc.COLOR_BGR2RGB); // Convert color channels
            Imgproc.resize(preprocessedFrame, preprocessedFrame, new Size(frame.width(), frame.height())); // Resize to model's input size
            preprocessedFrame.convertTo(preprocessedFrame, CvType.CV_32F); // Convert to float type
            return preprocessedFrame;
        }


        private void visualizeResults(Mat frame, List<DetectionResult> detectionResults) {
// Visualize the detection results on the frame using OpenCV drawing functions
            for (DetectionResult result : detectionResults) {
// Extract bounding box coordinates, class label, and confidence score from the DetectionResult
                Rect bbox = result.getBoundingBox();
                String label = result.getClassLabel();
                float score = result.getConfidenceScore();
                Imgproc.rectangle(frame, bbox.tl(), bbox.br(), new Scalar(0, 255, 0), 2);
                Imgproc.putText(frame, label + ": " + score, bbox.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 255, 0), 2);
            }
        }
    }
}

