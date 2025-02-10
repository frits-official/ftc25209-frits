package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

public class DetectSampleProcessor {
    private Camera camera;
    private ColorBlobLocatorProcessor colorLocator;
    private ColorRange color;
    private ArrayList<Double> angles = new ArrayList<Double>();

    public DetectSampleProcessor(Camera camera, ColorRange color) {
        this.camera = camera;
        this.color = color;
    }

    /**
     * Build sample processor
     */
    public void initSampleProcessor() {
        this.colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setTargetColorRange(this.color)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        camera.addProcessorToQueue(this.colorLocator);
    }

    /**
     * Run detection process
     */
    public void process() {
        angles.clear();
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            double angle = boxFit.angle;
            if (boxFit.size.width < boxFit.size.height) {
                angle += 90;
            }
            angles.add(angle);
        }
    }

    /**
     * Get angle list of detected sample
     */
    public ArrayList<Double> getAngles() {
        return angles;
    }
}
