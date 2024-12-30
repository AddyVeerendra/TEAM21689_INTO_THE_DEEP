package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;

public class SampleDetectionPipeline extends OpenCvPipeline {
    private Mat hsvFrame = new Mat();
    private Mat yellowMask = new Mat();
    private Mat redMask1 = new Mat();
    private Mat redMask2 = new Mat();
    private Mat combinedRedMask = new Mat();
    private Mat blueMask = new Mat();
    private Mat hierarchy = new Mat();
    private List<Sample> detectedSamples = new CopyOnWriteArrayList<>();
    private Map<String, Integer> persistentSamples = new HashMap<>();

    // Custom class to store sample data
    public static class Sample {
        public String color;
        public double area;
        public double centerX;
        public double centerY;

        public Sample(String color, double area, double centerX, double centerY) {
            this.color = color;
            this.area = area;
            this.centerX = centerX;
            this.centerY = centerY;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Apply Gaussian Blur to reduce noise
        Imgproc.GaussianBlur(input, input, new org.opencv.core.Size(5, 5), 0);

        // Convert the input frame from RGB to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Define the tighter HSV range for yellow
        Scalar lowerYellow = new Scalar(22, 150, 150);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Define the HSV range for red
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(170, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Define the tighter HSV range for blue
        Scalar lowerBlue = new Scalar(100, 170, 170);
        Scalar upperBlue = new Scalar(120, 255, 255);

        // Create masks for yellow, red, and blue areas in the frame
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        // Use morphological operations to refine masks
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.erode(redMask1, redMask1, kernel);
        Imgproc.erode(redMask2, redMask2, kernel);

        // Combine the two red masks
        Core.bitwise_or(redMask1, redMask2, combinedRedMask);

        // Weighted filter to refine overlapping regions (e.g., yellow vs. red)
        Mat weightedMask = new Mat();
        Core.addWeighted(yellowMask, 0.5, combinedRedMask, 0.5, 0, weightedMask);
        Core.inRange(weightedMask, new Scalar(128), new Scalar(255), yellowMask);

        // Clear the list of detected samples
        detectedSamples.clear();

        // Detect samples for each color
        detectColorSamples(input, yellowMask, "Yellow", new Scalar(255, 255, 0)); // Yellow in BGR
        detectColorSamples(input, combinedRedMask, "Red", new Scalar(255, 0, 0)); // Red in BGR
        detectColorSamples(input, blueMask, "Blue", new Scalar(0, 0, 255));       // Blue in BGR

        // Return the processed frame with rectangles drawn
        return input;
    }

    private void detectColorSamples(Mat input, Mat mask, String colorName, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Calculate adaptive area thresholds
        int minArea = (int) (input.rows() * input.cols() * 0.001); // 0.1% of frame area
        int maxArea = (int) (input.rows() * input.cols() * 0.1);   // 10% of frame area

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);

            // Use adaptive area filtering
            if (area > minArea && area < maxArea) {
                Imgproc.rectangle(input, boundingRect, boxColor, 3);

                // Calculate center coordinates
                double centerX = boundingRect.x + boundingRect.width / 2.0;
                double centerY = boundingRect.y + boundingRect.height / 2.0;

                // Add detection to list and persistent sample tracking
                detectedSamples.add(new Sample(colorName, area, centerX, centerY));
                persistentSamples.put(colorName, persistentSamples.getOrDefault(colorName, 0) + 1);
            }
        }
    }

    // Method to return confirmed samples after multi-frame validation
    public List<Sample> getConfirmedSamples() {
        List<Sample> confirmedSamples = new ArrayList<>();
        for (Sample sample : detectedSamples) {
            if (persistentSamples.getOrDefault(sample.color, 0) >= 3) { // Appear in 3+ frames
                confirmedSamples.add(sample);
            }
        }
        return confirmedSamples;
    }

    // Release resources to avoid memory leaks
    @Override
    public void onViewportTapped() {
        hsvFrame.release();
        yellowMask.release();
        redMask1.release();
        redMask2.release();
        combinedRedMask.release();
        blueMask.release();
        hierarchy.release();
    }
}
