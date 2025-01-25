package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

public class SamplePNP_Pipeline extends OpenCvPipeline {

    private final Scalar YELLOW_LOWER = new Scalar(20, 100, 150);
    private final Scalar YELLOW_UPPER = new Scalar(30, 255, 255);
    private final Scalar BLUE_LOWER = new Scalar(100, 100, 50);
    private final Scalar BLUE_UPPER = new Scalar(130, 255, 255);
    private final Scalar RED_LOWER1 = new Scalar(0, 100, 100);
    private final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    private final Scalar RED_LOWER2 = new Scalar(170, 100, 100);
    private final Scalar RED_UPPER2 = new Scalar(180, 255, 255);

    private final double focalLength;  // Camera's focal length
    private final double realObjectHeight;  // Real height of the object to estimate distance

    private Mat processedFrame = new Mat(); // For visualization
    private List<Sample> detectedSamples = new ArrayList<>();

    // Constructor
    public SamplePNP_Pipeline(double focalLength, double realObjectHeight) {
        this.focalLength = focalLength;
        this.realObjectHeight = realObjectHeight;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Clear previous samples
        detectedSamples.clear();

        // Convert to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Create masks for each color
        Mat yellowMask = createColorMask(hsvImage, YELLOW_LOWER, YELLOW_UPPER);
        Mat redMask = new Mat();
        Core.bitwise_or(createColorMask(hsvImage, RED_LOWER1, RED_UPPER1),
                createColorMask(hsvImage, RED_LOWER2, RED_UPPER2), redMask);
        Mat blueMask = createColorMask(hsvImage, BLUE_LOWER, BLUE_UPPER);

        // Process each mask
        processColorMask(yellowMask, "Yellow", input);
        processColorMask(redMask, "Red", input);
        processColorMask(blueMask, "Blue", input);

        // Optional visualization: Draw bounding boxes
        for (Sample sample : detectedSamples) {
            Scalar color = sample.color.equals("Yellow") ? new Scalar(255, 255, 0) :
                    sample.color.equals("Red") ? new Scalar(255, 0, 0) :
                            new Scalar(0, 0, 255);
            Point topLeft = new Point(sample.boundingBox.x, sample.boundingBox.y);
            Point bottomRight = new Point(sample.boundingBox.x + sample.boundingBox.width,
                    sample.boundingBox.y + sample.boundingBox.height);
            Imgproc.rectangle(input, topLeft, bottomRight, color, 2);
        }

        // Release resources
        hsvImage.release();
        yellowMask.release();
        redMask.release();
        blueMask.release();

        // Return the modified frame
        processedFrame = input;
        return processedFrame;
    }

    private Mat createColorMask(Mat hsvImage, Scalar lower, Scalar upper) {
        Mat mask = new Mat();
        Core.inRange(hsvImage, lower, upper, mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        return mask;
    }

    private void processColorMask(Mat mask, String colorName, Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < input.size().area() * 0.001 || area > input.size().area() * 0.1) {
                continue;
            }

            Rect boundingBox = Imgproc.boundingRect(contour);
            double centerX = boundingBox.x + boundingBox.width / 2.0;
            double centerY = boundingBox.y + boundingBox.height / 2.0;
            double displacementX = centerX - input.width() / 2.0;
            double displacementY = centerY - input.height() / 2.0;

            // Calculate distance using focal length and bounding box height
            double distance = focalLength / boundingBox.height * realObjectHeight;
            double rotation = 0; // Placeholder for rotation logic

            // Add the detected sample
            detectedSamples.add(new Sample(colorName, distance, displacementX, displacementY, rotation, boundingBox));
        }

        hierarchy.release();
    }

    public List<Sample> getDetectedSamples() {
        return detectedSamples;
    }

    public Sample getHighestRankedSample() {
        return detectedSamples.stream()
                .max((s1, s2) -> Double.compare(s1.rank, s2.rank))
                .orElse(null);
    }

    // Internal class for storing detected samples
    static class Sample {
        String color;
        double distance;
        double displacementX;
        double displacementY;
        double rotation;
        double rank;
        Rect boundingBox;

        public Sample(String color, double distance, double displacementX, double displacementY, double rotation, Rect boundingBox) {
            this.color = color;
            this.distance = distance;
            this.displacementX = displacementX;
            this.displacementY = displacementY;
            this.rotation = rotation;
            this.rank = 0;  // Default rank, can be set later
            this.boundingBox = boundingBox;
        }

        @Override
        public String toString() {
            return String.format("Sample(color=%s, distance=%.2f, rank=%.2f)", color, distance, rank);
        }
    }
}
