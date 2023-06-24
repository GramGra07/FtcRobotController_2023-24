package org.firstinspires.ftc.teamcode.opModes.camera.openCV.ObjectDetection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.tensorflow.lite.Interpreter;

import java.io.IOException;

public class TFODBuilder {
    private final HardwareMap map;
    private final String modelName;
    private boolean quantized;
    private boolean drawOnImage;
    private float minResultConfidence = 0.6f;
    private final Interpreter.Options options;
    private String[] labels;

    public TFODBuilder(HardwareMap map, String modelName, String... labels){
        this.map = map;
        this.modelName = modelName;
        quantized = false;
        options = new Interpreter.Options();
        this.labels = labels;
        options.setCancellable(true);
        this.drawOnImage = true;
    }

    /**
     * Sets if the model is quantized
     *
     * Quantization of a model processes the model so that it uses integer inputs instead of floating point inputs
     * This speeds up inference on the model, but also (slightly) decreases accuracy
     */
    public TFODBuilder setQuantized(boolean quantized){
        this.quantized = quantized;
        return this;
    }

    /**
     * Sets the labels that the model uses.
     *
     * Deprecated: Set the labels in the constructor instead
     *
     * This is the labels that the model use, like "ring" or "goal"
     */
    @Deprecated
    public TFODBuilder setLabels(String... labels){
        this.labels = labels;
        return this;
    }

    /**
     * Number of threads that the model will use
     *
     * Generally, it is recommended to use somewhere between 1-4 threads
     * On the Control Hub, it seems the most optimal number of threads is somewhere between 2-3
     * The optimal number of threads may change depending on the model
     */
    public TFODBuilder setNumThreads(int numThreads){
        this.options.setNumThreads(numThreads);
        return this;
    }

    /**
     * Sets if the XNNPack delegate is used
     *
     * XNNPack is a set of neural network operators that are highly optimized for running floating point models on ARM, x86, and WebAssembly
     * This has the potential to run floating point (i.e non-quantized) models faster
     */
    public TFODBuilder useXNNPack(boolean xnnPack){
        this.options.setUseXNNPACK(xnnPack);
        return this;
    }

    /**
     * Uses a NNAPI delegate
     *
     * NNAPI utilizes dedicated hardware accelerators, such as Graphics Processing Units, Digital Signal Processors, or Neural Processing Units to speed up model inference
     * NNAPI requires backend architecture to allow for acceleration
     * The FTC Control Hub seems to allow for NNAPI acceleration
     */
    public TFODBuilder useNNAPI(){ //TODO: Test if the FTC Control Hub supports NNAPI acceleration (initial tests seems it doesn't?)
        this.options.setUseNNAPI(true);
        return this;
    }

    /**
     * Sets if the buffer handle output is used
     *
     * By default, when using hardware acceleration, data is copied to the CPU buffer before reading
     * Setting this value to FALSE allows for the device to attempt to read data directly from the hardware accelerated buffers
     * This is not supported on all devices but may give an advantage in inference time when used on a supported device
     */
    public TFODBuilder allowBufferHandleOutput(boolean allow){
        this.options.setAllowBufferHandleOutput(allow);
        return this;
    }

    /**
     * Sets if the API should draw detected objects on the input bitmap
     *
     * Warning: Can slow performance if lots of rectangles are being drawn
     */
    public TFODBuilder drawOnImage(boolean drawOnImage){
        this.drawOnImage = drawOnImage;
        return this;
    }

    /**
     * Sets the minimum result confidence to keep an object result
     * TFOD can sometimes return a lot of garbage detections with really low scores
     * So this helps filter them out
     * Confidence should be a float between 0 and 1, wtih 1 being 100% confidence and 0 being 0%
     * Default is 0.6 (60% confidence)
     */
    public TFODBuilder setMinResultConfidence(float minResultConfidence) {
        this.minResultConfidence = minResultConfidence;
        return this;
    }

    public TensorObjectDetector build() throws IOException {
        return new TensorObjectDetector(map, modelName, quantized, drawOnImage, minResultConfidence, options, labels);
    }
}
