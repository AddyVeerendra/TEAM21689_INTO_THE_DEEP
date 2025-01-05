package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.CLAHE;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class SamplePNP_Pipeline extends OpenCvPipeline {

    private final Telemetry telemetry;
    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private Mat labImage = new Mat();
    private Mat lChannel = new Mat();

    private static final Scalar YELLOW_LOWER = new Scalar(22, 150, 150);
    private static final Scalar YELLOW_UPPER = new Scalar(30, 255, 255);

    private static final Scalar BLUE_LOWER = new Scalar(100, 170, 170);
    private static final Scalar BLUE_UPPER = new Scalar(120, 255, 255);

    private static final Scalar RED_LOWER1 = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER2 = new Scalar(170, 100, 100);
    private static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);

    private final double focalLength;
    private final double realObjectHeight;
    private final List<Sample> detectedSamples = new ArrayList<>();

    public SamplePNP_Pipeline(Telemetry telemetry, double focalLength, double realObjectHeight) {
        this.telemetry = telemetry;
        this.focalLength = focalLength;
        this.realObjectHeight = realObjectHeight;
    }

    @Override
    public Mat processFrame(Mat input) {
        detectedSamples.clear();
        // Step 1: Convert the image to LAB color space
        Imgproc.cvtColor(input, labImage, Imgproc.COLOR_BGR2Lab);

        // Step 2: Split the LAB image into its channels
        List<Mat> labChannels = new ArrayList<>(3);
        Core.split(labImage, labChannels);
        lChannel = labChannels.get(0);

        // Step 3: Apply CLAHE to the L channel
        CLAHE clahe = Imgproc.createCLAHE();
        clahe.setClipLimit(2.0);
        clahe.apply(lChannel, lChannel);

        // Step 4: Merge the LAB channels back
        Core.merge(labChannels, labImage);

        // Step 5: Convert the LAB image back to BGR
        Imgproc.cvtColor(labImage, input, Imgproc.COLOR_Lab2BGR);

        // Step 6: Convert the image to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Step 7: Create masks for each color
        Mat yellowMask = createColorMask(hsvImage, YELLOW_LOWER, YELLOW_UPPER);
        Mat blueMask = new Mat();
        Core.bitwise_or(
                createColorMask(hsvImage, RED_LOWER1, RED_UPPER1),
                createColorMask(hsvImage, RED_LOWER2, RED_UPPER2),
                blueMask
        );
        Mat redMask = createColorMask(hsvImage, BLUE_LOWER, BLUE_UPPER);

        // Step 8: Process masks to find contours
        processColorMask(yellowMask, input, "Yellow");
        processColorMask(redMask, input, "Red");
        processColorMask(blueMask, input, "Blue");

        // Step 9: Release memory for temporary masks
        //kernel.release();

        // Step 10: Return the processed frame
        return input;
    }

    private Mat createColorMask(Mat hsvImage, Scalar lower, Scalar upper) {
        Core.inRange(hsvImage, lower, upper, mask);
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        return mask.clone();
    }

    private void processColorMask(Mat mask, Mat input, String colorName) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int minArea = (int) (input.rows() * input.cols() * 0.001); // 0.1% of frame area
        int maxArea = (int) (input.rows() * input.cols() * 0.1);

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < minArea || contourArea > maxArea) {
                continue; // Skip contours that are too small or too large
            }

            Rect boundingRect = Imgproc.boundingRect(contour);
            double x = boundingRect.x;
            double y = boundingRect.y;
            double w = boundingRect.width;
            double h = boundingRect.height;

            // Calculate center coordinates
            double centerX = x + w / 2.0;
            double centerY = y + h / 2.0;

            // Calculate the center of the screen
            double screenCenterX = input.cols() / 2.0;
            double screenCenterY = input.rows() / 2.0;

            // Calculate the displacement from the center of the screen
            double displacementX = centerX - screenCenterX;
            double displacementY = centerY - screenCenterY;

            // Assume the bounding box corners are image points
            MatOfPoint2f imagePoints = new MatOfPoint2f(
                    new Point(x, y),
                    new Point(x + w, y),
                    new Point(x + w, y + h),
                    new Point(x, y + h)
            );

            // Define 3D object points (real-world dimensions in cm)
            MatOfPoint3f objectPoints = new MatOfPoint3f(
                    new Point3(0, 0, 0),
                    new Point3(1.5, 0, 0),
                    new Point3(1.5, 8.9, 0),
                    new Point3(0, 8.9, 0)
            );

            Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_32F);
            cameraMatrix.put(0, 0, focalLength, 0, input.cols() / 2.0);
            cameraMatrix.put(1, 1, focalLength, input.rows() / 2.0);

            Mat rvec = new Mat();
            Mat tvec = new Mat();

            boolean success = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, new MatOfDouble(), rvec, tvec);

            if (success) {
                double distance = Math.sqrt(
                        tvec.get(0, 0)[0] * tvec.get(0, 0)[0] +
                                tvec.get(1, 0)[0] * tvec.get(1, 0)[0] +
                                tvec.get(2, 0)[0] * tvec.get(2, 0)[0]
                );

                Imgproc.rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, colorName + String.format(" %.2f cm", distance),
                        new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);

                detectedSamples.add(new Sample(colorName, distance, displacementX, displacementY));
            }
        }
    }

    public List<Sample> getDetectedSamples() {
        return detectedSamples;
    }

    public static class Sample {
        public final String color;
        public final double distance;
        public final double displacementX;
        public final double displacementY;

        public Sample(String color, double distance, double displacementX, double displacementY) {
            this.color = color;
            this.distance = distance;
            this.displacementX = displacementX;
            this.displacementY = displacementY;
        }
    }

    @Override
    public void onViewportTapped() {
        // Release resources held by Mat objects
        hsvImage.release();
        mask.release();
        kernel.release();
        labImage.release();
        lChannel.release();

        // Clear the detected samples list
        detectedSamples.clear();

        // Log or provide telemetry feedback
        telemetry.addData("Viewport", "Tapped and resources released");
        telemetry.update();
    }
}