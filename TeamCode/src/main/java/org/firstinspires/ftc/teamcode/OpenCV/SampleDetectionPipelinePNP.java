package org.firstinspires.ftc.teamcode.OpenCV;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class SampleDetectionPipelinePNP extends OpenCvPipeline
{
    /*
     * Our working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int YELLOW_MASK_THRESHOLD = 57;
    static final int BLUE_MASK_THRESHOLD = 150;
    static final int RED_MASK_THRESHOLD = 198;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);

    static final int CONTOUR_LINE_THICKNESS = 2;

    static class AnalyzedStone
    {
        double angle;
        String color;
        Mat rvec;
        Mat tvec;
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS
    }

    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    public SampleDetectionPipelinePNP()
    {
        // Initialize camera parameters
        // Replace these values with your actual camera calibration parameters

        // Focal lengths (fx, fy) and principal point (cx, cy)
        double fx = 822.317; // Replace with your camera's focal length in pixels
        double fy = 822.317;
        double cx = 319.495; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 242.502; // Replace with your camera's principal point y-coordinate (usually image height / 2)

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // If you have calibrated your camera and have these values, use them
        // Otherwise, you can assume zero distortion for simplicity
        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    @Override
    public void onViewportTapped()
    {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // We'll be updating this with new data below
        internalStoneList.clear();

        /*
         * Run the image processing
         */
        findContours(input);

        clientStoneList = new ArrayList<>(internalStoneList);

        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum])
        {
            case YCrCb:
            {
                return ycrcbMat;
            }

            case FINAL:
            {
                return input;
            }

            case MASKS:
            {
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;
            }

            case MASKS_NR:
            {
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;
            }

            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }
        }

        return input;
    }

    public ArrayList<AnalyzedStone> getDetectedStones()
    {
        return clientStoneList;
    }

    void findContours(Mat input) {
        // Downscale the input image to reduce computational load
        Mat resizedInput = new Mat();
        Imgproc.resize(input, resizedInput, new Size((double) input.width() / 2, input.height() / 2));

        // Convert the resized input image to YCrCb color space
        Imgproc.cvtColor(resizedInput, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb and Cr channels for blue and red detection
        Core.extractChannel(ycrcbMat, cbMat, 2); // Cb channel index is 2
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        // Apply Gaussian blur to reduce noise
        Imgproc.GaussianBlur(cbMat, cbMat, new Size(5, 5), 0);
        Imgproc.GaussianBlur(crMat, crMat, new Size(5, 5), 0);

        // Adaptive thresholding for blue and red masks
        Imgproc.adaptiveThreshold(cbMat, blueThresholdMat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);
        Imgproc.adaptiveThreshold(crMat, redThresholdMat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);

        // Threshold the Cb channel to form a mask for yellow
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks with hierarchy
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Mat hierarchyBlue = new Mat();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, hierarchyBlue, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Mat hierarchyRed = new Mat();
        Imgproc.findContours(morphedRedThreshold, redContoursList, hierarchyRed, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Mat hierarchyYellow = new Mat();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, hierarchyYellow, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // Analyze the contours with hierarchy
        analyzeContoursWithHierarchy(blueContoursList, hierarchyBlue, resizedInput, "Blue");
        analyzeContoursWithHierarchy(redContoursList, hierarchyRed, resizedInput, "Red");
        analyzeContoursWithHierarchy(yellowContoursList, hierarchyYellow, resizedInput, "Yellow");

        // Release the resized input Mat
        resizedInput.release();
    }

    void analyzeContoursWithHierarchy(ArrayList<MatOfPoint> contours, Mat hierarchy, Mat input, String color) {
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > 100) { // Filter small contours
                int[] hierarchyData = new int[4];
                hierarchy.get(0, i, hierarchyData);
                if (hierarchyData[3] == -1) { // Only consider outermost contours
                    analyzeContour(contours.get(i), input, color);
                }
            }
        }
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input, color);

        // The angle OpenCV gives us can be ambiguous, so look at the shape of
        // the rectangle to fix that.
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
        {
            rotRectAngle += 90;
        }

        // Compute the angle and store it
        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle)) + " deg", input, color);

        // Prepare object points and image points for solvePnP
        // Assuming the object is a rectangle with known dimensions
        double objectWidth = 10.0;  // Replace with your object's width in real-world units (e.g., centimeters)
        double objectHeight = 5.0;  // Replace with your object's height in real-world units

        // Define the 3D coordinates of the object corners in the object coordinate space
        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        // Get the 2D image points from the detected rectangle corners
        Point[] rectPoints = new Point[4];
        rotatedRectFitToContour.points(rectPoints);

        // Order the image points in the same order as object points
        Point[] orderedRectPoints = orderPoints(rectPoints);

        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

        // Solve PnP
        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(
                objectPoints, // Object points in 3D
                imagePoints,  // Corresponding image points
                cameraMatrix,
                distCoeffs,
                rvec,
                tvec
        );

        if (success)
        {
            // Draw the coordinate axes on the image
            drawAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            // Store the pose information
            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = rotRectAngle;
            analyzedStone.color = color;
            analyzedStone.rvec = rvec;
            analyzedStone.tvec = tvec;
            internalStoneList.add(analyzedStone);
        }
    }

    void drawAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs)
    {
        // Length of the axis lines
        double axisLength = 5.0;

        // Define the points in 3D space for the axes
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axisLength, 0, 0),
                new Point3(0, axisLength, 0),
                new Point3(0, 0, -axisLength) // Z axis pointing away from the camera
        );

        // Project the 3D points to 2D image points
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        // Draw the axis lines
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2); // X axis in red
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2); // Y axis in green
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2); // Z axis in blue
    }

    static Point[] orderPoints(Point[] pts)
    {
        // Orders the array of 4 points in the order: top-left, top-right, bottom-right, bottom-left
        Point[] orderedPts = new Point[4];

        // Sum and difference of x and y coordinates
        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++)
        {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        // Top-left point has the smallest sum
        int tlIndex = indexOfMin(sum);
        orderedPts[0] = pts[tlIndex];

        // Bottom-right point has the largest sum
        int brIndex = indexOfMax(sum);
        orderedPts[2] = pts[brIndex];

        // Top-right point has the smallest difference
        int trIndex = indexOfMin(diff);
        orderedPts[1] = pts[trIndex];

        // Bottom-left point has the largest difference
        int blIndex = indexOfMax(diff);
        orderedPts[3] = pts[blIndex];

        return orderedPts;
    }

    static int indexOfMin(double[] array)
    {
        int index = 0;
        double min = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] < min)
            {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array)
    {
        int index = 0;
        double max = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] > max)
            {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color)
    {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color)
    {
        switch (color)
        {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }
}