package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
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
        // Convert the input frame from RGB to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Define the tighter HSV range for yellow (more saturated and bright)
        Scalar lowerYellow = new Scalar(22, 150, 150); // Increased saturation and value
        Scalar upperYellow = new Scalar(30, 255, 255); // Keeping the upper bounds tight for yellow

        // Define the HSV range for red
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(170, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Define the tighter HSV range for blue (more saturated and bright)
        Scalar lowerBlue = new Scalar(100, 170, 170);  // Increased saturation and value thresholds
        Scalar upperBlue = new Scalar(120, 255, 255);  // Narrower hue range for blue

        // Create masks for yellow, red, and blue areas in the frame
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        // Combine the two red masks using bitwise_or
        Core.bitwise_or(redMask1, redMask2, combinedRedMask);

        // Clear the list of detected samples
        detectedSamples.clear();

        // Correct the color values for BGR format
        detectColorSamples(input, yellowMask, "Yellow", new Scalar(255, 255, 0)); // Yellow in BGR
        detectColorSamples(input, combinedRedMask, "Red", new Scalar(255, 0, 0)); // Red in BGR
        detectColorSamples(input, blueMask, "Blue", new Scalar(0, 0, 255));       // Blue in BGR

        // Return the processed frame with rectangles drawn
        return input;
    }

    private void detectColorSamples(Mat input, Mat mask, String colorName, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);

            // If the area is large enough, consider it a valid sample and draw a rectangle
            if (area > 300 && area < 25000) {  // Lower area threshold
                // Draw rectangle with color corresponding to detected sample's color
                Imgproc.rectangle(input, boundingRect, boxColor, 3);  // Use the color of the detected sample

                // Calculate the center of the bounding rectangle
                double centerX = boundingRect.x + boundingRect.width / 2.0;
                double centerY = boundingRect.y + boundingRect.height / 2.0;

                // Add the detected sample to the list with color, area, and center coordinates
                detectedSamples.add(new Sample(colorName, area, centerX, centerY));
            }
        }
    }

    // Method to return all detected samples as a list of Sample objects
    public List<Sample> getDetectedSamples() {
        return detectedSamples;
    }
}