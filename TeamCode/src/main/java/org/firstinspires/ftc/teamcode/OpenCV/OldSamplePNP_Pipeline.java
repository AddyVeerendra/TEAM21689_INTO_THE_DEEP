package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class OldSamplePNP_Pipeline extends OpenCvPipeline {

    private final Telemetry telemetry;
    private final List<Sample> detectedSamples;

    // Preallocated Mats (reused across frames)
    private final Mat hsvImage = new Mat();
    private final Mat labImage = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private final Mat lChannel = new Mat(); // (unused but retained for similarity)
    private Mat hsvFrame = new Mat();
    private Mat yellowMask = new Mat();
    private Mat redMask1 = new Mat();
    private Mat redMask2 = new Mat();
    private Mat combinedRedMask = new Mat();
    private Mat blueMask = new Mat();
    private Mat hierarchy = new Mat();

    // Color range constants
    private static final Scalar RED_LOWER1 = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);

    private static final Scalar RED_LOWER2 = new Scalar(170, 100, 100);
    private static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(22, 150, 150);
    private static final Scalar YELLOW_UPPER  = new Scalar(39, 255, 255);
    private static final Scalar BLUE_LOWER = new Scalar(100, 150, 50);
    private static final Scalar BLUE_UPPER = new Scalar(140, 255, 255);

    // Calibration parameters from the webcam
    // Image size: 640 x 480
    // Focal lengths and principal point are used to build the camera matrix.
    // Note: If using a resolution different from 640x480, you'll need to scale these values.
    private static final float CALIBRATED_FOCAL_X = 822.317f;
    private static final float CALIBRATED_FOCAL_Y = 822.317f;
    private static final float CALIBRATED_PRINCIPAL_X = 319.495f;
    private static final float CALIBRATED_PRINCIPAL_Y = 242.502f;
    // Provided eight distortion coefficients; typically five are used.
    // Here we pass all eight for completeness.
    private static final double[] CALIBRATED_DIST_COEFFS = new double[]{
            -0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0
    };

    // Fixed camera matrix and distortion coefficients built from the calibration parameters.
    private final Mat cameraMatrix;
    private final MatOfDouble distCoeffs;

    private final double focalLength;      // (Not actively used anymore, replaced by calibration data)
    private final double realObjectHeight; // (Not actively used in this snippet)

    // Precomputed 3D object points (in cm) and the object's center.
    // Order: top-left, top-right, bottom-right, bottom-left.
    private final MatOfPoint3f objectPoints;
    private final Mat centerObject;

    public OldSamplePNP_Pipeline(double focalLength, double realObjectHeight, Telemetry telemetry) {
        this.focalLength = focalLength;
        this.realObjectHeight = realObjectHeight;
        this.telemetry = telemetry;
        this.detectedSamples = new ArrayList<>();

        // Build the camera matrix using the calibrated parameters.
        cameraMatrix = Mat.zeros(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, CALIBRATED_FOCAL_X, 0, CALIBRATED_PRINCIPAL_X,
                0, CALIBRATED_FOCAL_Y, CALIBRATED_PRINCIPAL_Y,
                0, 0, 1);

        // Build the distortion coefficients from the calibrated parameters.
        distCoeffs = new MatOfDouble(CALIBRATED_DIST_COEFFS);

        // Precompute the 3D object points (in cm) for a rectangular object.
        // The order is: top-left, top-right, bottom-right, bottom-left.
        objectPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),       // Top-left
                new Point3(1.5, 0, 0),     // Top-right
                new Point3(1.5, 8.9, 0),   // Bottom-right
                new Point3(0, 8.9, 0)      // Bottom-left
        );
        // Precompute the object's center in 3D (in cm): (1.5/2, 8.9/2, 0) = (0.75, 4.45, 0)
        centerObject = new Mat(3, 1, CvType.CV_64F);
        centerObject.put(0, 0, 1.5 / 2.0);
        centerObject.put(1, 0, 8.9 / 2.0);
        centerObject.put(2, 0, 0);
    }

    @Override
    public Mat processFrame(Mat input) {
        // (Since we now have a fixed calibration, we assume the resolution is 640x480.)
        // If you plan to support other resolutions, add scaling logic here.

        detectedSamples.clear();

        Imgproc.GaussianBlur(input, input, new org.opencv.core.Size(5, 5), 0);

        // Convert to HSV
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Create color masks (reuse "mask" and clone the result)
        Core.inRange(hsvImage, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
        Core.inRange(hsvImage, RED_LOWER1, RED_UPPER1, redMask1);
        Core.inRange(hsvImage, RED_LOWER2, RED_UPPER2, redMask2);
        Core.inRange(hsvImage, BLUE_LOWER, BLUE_UPPER, blueMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.erode(redMask1, redMask1, kernel);
        Imgproc.erode(redMask2, redMask2, kernel);

        Core.bitwise_or(redMask1, redMask2, combinedRedMask);

        detectedSamples.clear();

        // Process each color mask.
        processColorMask(yellowMask, input, "Yellow");
        processColorMask(combinedRedMask, input, "Red");
        processColorMask(blueMask, input, "Blue");

        // Release temporary mask Mats.
        kernel.release();

        return input;
    }

    /**
     * Creates a color mask based on HSV thresholds.
     */
    private Mat createColorMask(Mat hsv, Scalar lower, Scalar upper) {
        Core.inRange(hsv, lower, upper, mask);
        return mask.clone();
    }

    /**
     * Orders an array of 4 points into the following order:
     * Top-left, Top-right, Bottom-right, Bottom-left.
     */
    private MatOfPoint2f orderPoints(Point[] points) {
        if (points.length != 4) {
            throw new IllegalArgumentException("There must be exactly 4 points.");
        }

        // Convert array to a list for easier sorting.
        List<Point> pointList = new ArrayList<>(Arrays.asList(points));

        // Sort by y-coordinate (ascending) to separate top points from bottom points.
        Collections.sort(pointList, Comparator.comparingDouble(p -> p.y));

        // The first two in the sorted list are the top points.
        List<Point> topPoints = pointList.subList(0, 2);
        // The last two are the bottom points.
        List<Point> bottomPoints = pointList.subList(2, 4);

        // For the top points, sort by x-coordinate so that left comes first.
        Collections.sort(topPoints, Comparator.comparingDouble(p -> p.x));
        // For the bottom points, sort by x-coordinate so that left comes first.
        Collections.sort(bottomPoints, Comparator.comparingDouble(p -> p.x));

        // Now assign in the order:
        // top-left, top-right, bottom-right, bottom-left
        Point topLeft = topPoints.get(0);
        Point topRight = topPoints.get(1);
        Point bottomLeft = bottomPoints.get(0);
        Point bottomRight = bottomPoints.get(1);

        Point[] ordered = new Point[]{topLeft, topRight, bottomRight, bottomLeft};
        return new MatOfPoint2f(ordered);
    }

    /**
     * Processes each color mask by finding contours, fitting a rotated rectangle,
     * and then using solvePnP to calculate object pose and displacements.
     */
    private void processColorMask(Mat colorMask, Mat input, String colorName) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        int minArea = (int) (input.rows() * input.cols() * 0.001);
        int maxArea = (int) (input.rows() * input.cols() * 0.1);

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < minArea || contourArea > maxArea) {
                contour.release();
                continue;
            }

            // Convert contour to MatOfPoint2f for accurate processing.
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            // Get the 4 vertices of the rotated rectangle.
            Point[] boxPoints = new Point[4];
            rotatedRect.points(boxPoints);

            // Order the points so that they match the order of the 3D model points.
            MatOfPoint2f imagePoints = orderPoints(boxPoints);

            // Draw the rotated rectangle using the ordered points.
            Point[] orderedPoints = imagePoints.toArray();
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, orderedPoints[i], orderedPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Use the rotated rectangle's center for computing pixel displacement.
            double centerX = rotatedRect.center.x;
            double centerY = rotatedRect.center.y;
            double screenCenterX = input.cols() / 2.0;
            double screenCenterY = input.rows() / 2.0;
            double displacementX = centerX - screenCenterX;
            double displacementY = centerY - screenCenterY;

            // Solve for pose using the precomputed 3D object points and the ordered image points.
            Mat rvec = new Mat();
            Mat tvec = new Mat();
            boolean success = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
            if (!success) {
                contour2f.release();
                imagePoints.release();
                contour.release();
                rvec.release();
                tvec.release();
                continue;
            }

            // Compute the Euclidean distance (in cm) from the translation vector.
            double distance = Math.sqrt(
                    Math.pow(tvec.get(0, 0)[0], 2) +
                            Math.pow(tvec.get(1, 0)[0], 2) +
                            Math.pow(tvec.get(2, 0)[0], 2)
            );

            // Convert rotation vector to a rotation matrix and extract Euler angles.
            Mat rotationMatrix = new Mat();
            Calib3d.Rodrigues(rvec, rotationMatrix);
            double[] eulerAngles = rotationMatrixToEulerAngles(rotationMatrix);

            // Transform the object's center into the camera coordinate system.
            Mat centerCam = new Mat();
            Core.gemm(rotationMatrix, centerObject, 1, tvec, 1, centerCam);
            double horizontalOffset = centerCam.get(0, 0)[0];
            double verticalOffset = centerCam.get(1, 0)[0];

            // Annotate the frame with the color, distance, and displacements.
            Imgproc.putText(input, colorName + String.format(" %.2f cm", distance),
                    new Point(orderedPoints[0].x, orderedPoints[0].y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
            Imgproc.putText(input, String.format("H: %.2f cm V: %.2f cm", horizontalOffset, verticalOffset),
                    new Point(orderedPoints[0].x, orderedPoints[0].y + 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 1);

            // Determine if the detected rectangle is oriented horizontally.
            boolean horizontal = rotatedRect.size.width > rotatedRect.size.height;
            detectedSamples.add(new Sample(colorName, distance, displacementX, displacementY,
                    horizontalOffset, verticalOffset, eulerAngles[2], 0, horizontal));

            // Release temporary Mats for this contour.
            contour2f.release();
            imagePoints.release();
            rvec.release();
            tvec.release();
            rotationMatrix.release();
            centerCam.release();
            contour.release();
        }
    }

    /**
     * Converts a rotation matrix into Euler angles.
     */
    private double[] rotationMatrixToEulerAngles(Mat R) {
        double sy = Math.sqrt(Math.pow(R.get(0, 0)[0], 2) + Math.pow(R.get(1, 0)[0], 2));
        boolean singular = sy < 1e-6;
        double x, y, z;
        if (!singular) {
            x = Math.atan2(R.get(2, 1)[0], R.get(2, 2)[0]);
            y = Math.atan2(-R.get(2, 0)[0], sy);
            z = Math.atan2(R.get(1, 0)[0], R.get(0, 0)[0]);
        } else {
            x = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
            y = Math.atan2(-R.get(2, 0)[0], sy);
            z = 0;
        }
        return new double[]{x, y, z};
    }

    /**
     * Optimized cluster ranking via an index-based double loop.
     */
    private void SampleClusterRank(List<Sample> samples) {
        int size = samples.size();
        for (int i = 0; i < size; i++) {
            Sample sample = samples.get(i);
            for (int j = i + 1; j < size; j++) {
                Sample other = samples.get(j);
                double d = Math.sqrt(Math.pow(sample.displacementX - other.displacementX, 2)
                        + Math.pow(sample.displacementY - other.displacementY, 2));
                if (d < 100) {
                    if (other.color.equals("yellow") || other.color.equals(sample.color)) {
                        sample.rank++;
                        other.rank++;
                    } else {
                        sample.rank--;
                        other.rank--;
                    }
                }
            }
        }
    }

    public List<Sample> getDetectedSamples() {
        SampleClusterRank(detectedSamples);
        Collections.sort(detectedSamples, Comparator.comparingInt(s -> s.rank));
        return detectedSamples;
    }

    /**
     * The Sample class now includes fields for horizontal and vertical displacements.
     */
    public static class Sample {
        public final String color;
        public final double distance;
        public final double displacementX;  // Pixel displacement from image center (x-axis)
        public final double displacementY;  // Pixel displacement from image center (y-axis)
        public final double horizontalDisplacement;  // (in cm)
        public final double verticalDisplacement;    // (in cm)
        public final double rotation;
        public int rank;
        public final boolean horizontal;

        public Sample(String color, double distance, double displacementX, double displacementY,
                      double horizontalDisplacement, double verticalDisplacement,
                      double rotation, int rank, boolean horizontal) {
            this.color = color;
            this.distance = distance;
            this.displacementX = displacementX;
            this.displacementY = displacementY;
            this.horizontalDisplacement = horizontalDisplacement;
            this.verticalDisplacement = verticalDisplacement;
            this.rotation = rotation;
            this.rank = rank;
            this.horizontal = horizontal;
        }
    }

    @Override
    public void onViewportTapped() {
        // Release resources held by Mats.
        hsvImage.release();
        labImage.release();
        mask.release();
        kernel.release();
        lChannel.release();
        detectedSamples.clear();
        telemetry.addData("Viewport", "Tapped and resources released");
        telemetry.update();
    }
}
