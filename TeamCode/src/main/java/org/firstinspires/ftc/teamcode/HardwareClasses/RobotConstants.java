package org.firstinspires.ftc.teamcode.HardwareClasses;

import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;

public final class RobotConstants {
    // Prevent instantiation
    private RobotConstants() {}

    // Define actual points on the field so you don't have to change them. Measure with a tape measure.
    public static final Point EXAMPLE_POINT = new Point(0, 0, Point.CARTESIAN);

    // Change the distance to whatever element you are measuring from.
    public static final Point TO_FRONT_ELEMENT = new Point(0, 10, Point.CARTESIAN);

    public static final double EXAMPLE_SERVO_POSITION = 0.5;
}