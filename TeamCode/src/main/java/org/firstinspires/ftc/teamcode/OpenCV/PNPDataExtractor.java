// PNPDataExtractor.java
package org.firstinspires.ftc.teamcode.OpenCV;

import java.util.List;

public class PNPDataExtractor {

    /**
     * Container class for the PnP results.
     */
    public static class SampleData {
        public final String color;
        public final double horizontalDisplacement;  // in centimeters
        public final double verticalDisplacement;    // in centimeters
        public final int rank;
        public final boolean isHorizontal;           // true if the object is oriented horizontally

        public SampleData(String color, double horizontalDisplacement, double verticalDisplacement, int rank, boolean isHorizontal) {
            this.color = color;
            this.horizontalDisplacement = horizontalDisplacement;
            this.verticalDisplacement = verticalDisplacement;
            this.rank = rank;
            this.isHorizontal = isHorizontal;
        }
    }

    /**
     * Extracts the PnP parameters from the given CameraManagerPNP instance.
     * This method retrieves the detected samples, selects the highest-ranked one,
     * and returns a SampleData object containing the color, horizontal & vertical displacement,
     * rank, and whether the sample is horizontally oriented.
     *
     * @param cameraManager an instance of CameraManagerPNP that has processed a frame.
     * @return a SampleData object with the necessary parameters, or null if no sample was detected.
     */
    public static SampleData extractPNPData(CameraManagerPNP cameraManager) {
        // Get the list of detected samples from the camera manager.
        List<OldSamplePNP_Pipeline.Sample> detectedSamples = cameraManager.getDetectedSamples();

        if (detectedSamples != null && !detectedSamples.isEmpty()) {
            // Retrieve the highest-ranked sample.
            OldSamplePNP_Pipeline.Sample bestSample = cameraManager.getHighestRankedSample();
            if (bestSample != null) {
                // Create and return a SampleData object containing the relevant parameters.
                return new SampleData(
                        bestSample.color,
                        bestSample.horizontalDisplacement,
                        bestSample.verticalDisplacement,
                        bestSample.rank,
                        bestSample.horizontal
                );
            }
        }
        // Return null if no valid sample is detected.
        return null;
    }
}
