package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SamplePNP_Pipeline extends OpenCvPipeline {

    private final Telemetry telemetry;
    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    private static final Scalar YELLOW_LOWER = new Scalar(20, 100, 100);
    private static final Scalar YELLOW_UPPER = new Scalar(30, 255, 255);
    private static final Scalar BLUE_LOWER1 = new Scalar(0, 100, 100);
    private static final Scalar BLUE_UPPER1 = new Scalar(10, 255, 255);
    private static final Scalar BLUE_LOWER2 = new Scalar(170, 100, 100);
    private static final Scalar BLUE_UPPER2 = new Scalar(180, 255, 255);
    private static final Scalar RED_LOWER = new Scalar(100, 150, 0);
    private static final Scalar RED_UPPER = new Scalar(140, 255, 255);

    private final double focalLength;
    private final double realObjectHeight;
    private final List<Sample> detectedSamples = new ArrayList<>();

    private static final double MIN_CONTOUR_AREA = 500.0; // Minimum contour area
    private static final double MAX_CONTOUR_AREA = 50000.0; // Maximum contour area

    public SamplePNP_Pipeline(Telemetry telemetry, double focalLength, double realObjectHeight) {
        this.telemetry = telemetry;
        this.focalLength = focalLength;
        this.realObjectHeight = realObjectHeight;
    }

    @Override
    public Mat processFrame(Mat input) {
        detectedSamples.clear();
        // Step 1: Preprocess the image
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Step 2: Create masks for each color
        Mat yellowMask = createColorMask(hsvImage, YELLOW_LOWER, YELLOW_UPPER);
        Mat blueMask = new Mat();
        Core.bitwise_or(
                createColorMask(hsvImage, BLUE_LOWER1, BLUE_UPPER1),
                createColorMask(hsvImage, BLUE_LOWER2, BLUE_UPPER2),
                blueMask
        );
        Mat redMask = createColorMask(hsvImage, RED_LOWER, RED_UPPER);

        // Step 3: Process masks to find contours
        processColorMask(yellowMask, input, "Yellow");
        processColorMask(redMask, input, "Red");
        processColorMask(blueMask, input, "Blue");

        // Step 4: Return the processed frame
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

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < MIN_CONTOUR_AREA || contourArea > MAX_CONTOUR_AREA) {
                continue; // Skip contours that are too small or too large
            }

            Rect boundingRect = Imgproc.boundingRect(contour);
            double x = boundingRect.x;
            double y = boundingRect.y;
            double w = boundingRect.width;
            double h = boundingRect.height;

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

                telemetry.addData(colorName + " Sample Distance", "%.2f cm", distance);
                Imgproc.rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, colorName + String.format(" %.2f cm", distance),
                        new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);

                detectedSamples.add(new Sample(colorName, distance));
            }
        }
    }

    public List<Sample> getDetectedSamples() {
        return detectedSamples;
    }

    public static class Sample {
        public final String color;
        public final double distance;

        public Sample(String color, double distance) {
            this.color = color;
            this.distance = distance;
        }
    }
}