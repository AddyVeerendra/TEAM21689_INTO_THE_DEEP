package org.firstinspires.ftc.teamcode.LimeLight;

import java.util.List;
public class DistanceEstimator {

    private static final double FOCAL_LENGTH = 621;

    // Known dimensions of the rectangular prism (in meters)
    public static final double REAL_WORLD_LENGTH = 0.089;  // 8.9 cm
    public static final double REAL_WORLD_WIDTH = 0.015;   // 1.5 cm
    public static final double REAL_WORLD_HEIGHT = 0.015;  // 1.5 cm

    // Method to estimate distance based on real-world diagonal size and detected diagonal in pixels
    public static double estimateDistanceFromDiagonal(double realWorldDiagonal, double objectDiagonalPixels) {
        return (realWorldDiagonal * FOCAL_LENGTH) / objectDiagonalPixels;
    }

    // Method to estimate distance based on real-world width and object width in pixels
    public static double estimateDistanceFromWidth(double realWorldWidth, double objectWidthPixels) {
        return (realWorldWidth * FOCAL_LENGTH) / objectWidthPixels;
    }

    // Method to estimate distance based on real-world height and object height in pixels
    public static double estimateDistanceFromHeight(double realWorldHeight, double objectHeightPixels) {
        return (realWorldHeight * FOCAL_LENGTH) / objectHeightPixels;
    }

    // Calculate the diagonal of the bounding box in pixels from the targetCorners
    public static double getObjectDiagonalInPixels(List<List<Double>> targetCorners) {
        double x1 = targetCorners.get(0).get(0);
        double y1 = targetCorners.get(0).get(1);
        double x2 = targetCorners.get(2).get(0);
        double y2 = targetCorners.get(2).get(1);
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));  // Pythagorean theorem for diagonal
    }

    // Calculate the object width in pixels from the targetCorners
    public static double getObjectWidthInPixels(List<List<Double>> targetCorners) {
        double x1 = targetCorners.get(0).get(0);
        double x2 = targetCorners.get(1).get(0);
        return Math.abs(x2 - x1);
    }

    // Calculate the object height in pixels from the targetCorners
    public static double getObjectHeightInPixels(List<List<Double>> targetCorners) {
        double y1 = targetCorners.get(0).get(1);
        double y2 = targetCorners.get(1).get(1);
        return Math.abs(y2 - y1);
    }

    /*
    public static double[] estimateDistanceWithPose(
            List<List<Double>> targetCorners,
            Mat cameraMatrix,
            Mat distCoeffs) {

        // Real-world coordinates of the object corners (meters)
        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),              // Bottom-left corner
                new Point3(REAL_WORLD_WIDTH, 0, 0),  // Bottom-right corner
                new Point3(REAL_WORLD_WIDTH, REAL_WORLD_HEIGHT, 0), // Top-right corner
                new Point3(0, REAL_WORLD_HEIGHT, 0) // Top-left corner
        );

        // Image coordinates of the detected corners
        MatOfPoint2f imagePoints = new MatOfPoint2f(
                new Point2f(targetCorners.get(0).get(0).floatValue(), targetCorners.get(0).get(1).floatValue()),
                new Point2f(targetCorners.get(1).get(0).floatValue(), targetCorners.get(1).get(1).floatValue()),
                new Point2f(targetCorners.get(2).get(0).floatValue(), targetCorners.get(2).get(1).floatValue()),
                new Point2f(targetCorners.get(3).get(0).floatValue(), targetCorners.get(3).get(1).floatValue())
        );

        Mat rvec = new Mat(); // Rotation vector
        Mat tvec = new Mat(); // Translation vector

        // Solve PnP
        Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        // Extract translation vector (distance from camera)
        double[] tvecArray = tvec.get(0, 0);
        return new double[]{tvecArray[0], tvecArray[1], tvecArray[2]};  // x, y, z distances
    }
    */
}
