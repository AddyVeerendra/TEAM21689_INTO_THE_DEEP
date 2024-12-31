package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SamplePNP_Pipeline extends OpenCvPipeline {

    // --- Helper Methods ---
    public static Mat colorMask(Mat image, Scalar lower, Scalar upper) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);
        return mask;
    }

    public static Mat applyMorphology(Mat mask) {
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        return mask;
    }

    public static Mat enhanceContrast(Mat image) {
        Mat ycrcb = new Mat();
        Imgproc.cvtColor(image, ycrcb, Imgproc.COLOR_BGR2YCrCb);
        List<Mat> channels = new ArrayList<>();
        Core.split(ycrcb, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, ycrcb);
        Mat result = new Mat();
        Imgproc.cvtColor(ycrcb, result, Imgproc.COLOR_YCrCb2BGR);
        return result;
    }

    public static Mat applyFilter(Mat image) {
        Mat result = new Mat();
        Imgproc.GaussianBlur(image, result, new Size(5, 5), 0);
        return result;
    }

    public static double calculateDistance(double focalLength, double realHeight, double pixelHeight) {
        return (pixelHeight != 0) ? (focalLength * realHeight) / pixelHeight : -1;
    }

    // --- Main Detection Function ---
    public static void detectSamples(Mat image, double focalLength, double realObjectHeight) {
        // Preprocessing
        image = enhanceContrast(image);
        image = applyFilter(image);

        // Define color ranges
        Scalar yellowLower = new Scalar(20, 100, 100), yellowUpper = new Scalar(30, 255, 255);
        Scalar redLower1 = new Scalar(0, 100, 100), redUpper1 = new Scalar(10, 255, 255);
        Scalar redLower2 = new Scalar(170, 100, 100), redUpper2 = new Scalar(180, 255, 255);
        Scalar blueLower = new Scalar(100, 150, 0), blueUpper = new Scalar(140, 255, 255);

        // Create masks
        Mat yellowMask = applyMorphology(colorMask(image, yellowLower, yellowUpper));
        Mat redMask1 = colorMask(image, redLower1, redUpper1);
        Mat redMask2 = colorMask(image, redLower2, redUpper2);
        Mat redMask = new Mat();
        Core.bitwise_or(redMask1, redMask2, redMask);
        redMask = applyMorphology(redMask);
        Mat blueMask = applyMorphology(colorMask(image, blueLower, blueUpper));

        // Process each mask
        processMask(image, yellowMask, "Yellow", focalLength, realObjectHeight);
        processMask(image, redMask, "Red", focalLength, realObjectHeight);
        processMask(image, blueMask, "Blue", focalLength, realObjectHeight);

        // Show the output
        Imgcodecs.imwrite("output.jpg", image);
        System.out.println("Results saved to output.jpg");
    }

    public static void processMask(Mat image, Mat mask, String color, double focalLength, double realHeight) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(image, rect, new Scalar(0, 255, 0), 2);

            // Calculate distance
            double distance = calculateDistance(focalLength, realHeight, rect.height);
            String label = String.format("%s %.2f cm", color, distance);
            System.out.println(label);

            // Draw the label
            Imgproc.putText(image, label, new Point(rect.x, rect.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
        }
    }

    // --- Main Function ---
    public static void main(String[] args) {
        String imagePath = "input.jpg"; // Input image path
        double focalLength = 800;       // Example focal length (pixels)
        double realObjectHeight = 8.9;  // Real object height in cm

        Mat image = Imgcodecs.imread(imagePath);
        if (image.empty()) {
            System.out.println("Error: Could not load image.");
            return;
        }

        detectSamples(image, focalLength, realObjectHeight);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Preprocessing
        Mat enhancedImage = enhanceContrast(input);
        Mat filteredImage = applyFilter(enhancedImage);

        // Define color ranges
        Scalar yellowLower = new Scalar(20, 100, 100), yellowUpper = new Scalar(30, 255, 255);
        Scalar redLower1 = new Scalar(0, 100, 100), redUpper1 = new Scalar(10, 255, 255);
        Scalar redLower2 = new Scalar(170, 100, 100), redUpper2 = new Scalar(180, 255, 255);
        Scalar blueLower = new Scalar(100, 150, 0), blueUpper = new Scalar(140, 255, 255);

        // Create masks
        Mat yellowMask = applyMorphology(colorMask(filteredImage, yellowLower, yellowUpper));
        Mat redMask1 = colorMask(filteredImage, redLower1, redUpper1);
        Mat redMask2 = colorMask(filteredImage, redLower2, redUpper2);
        Mat redMask = new Mat();
        Core.bitwise_or(redMask1, redMask2, redMask);
        redMask = applyMorphology(redMask);
        Mat blueMask = applyMorphology(colorMask(filteredImage, blueLower, blueUpper));

        // Process each mask
        double focalLength = 800;       // Example focal length (pixels)
        double realObjectHeight = 8.9;  // Real object height in cm
        processMask(filteredImage, yellowMask, "Yellow", focalLength, realObjectHeight);
        processMask(filteredImage, redMask, "Red", focalLength, realObjectHeight);
        processMask(filteredImage, blueMask, "Blue", focalLength, realObjectHeight);

        // Return the processed frame
        return filteredImage;
    }
}