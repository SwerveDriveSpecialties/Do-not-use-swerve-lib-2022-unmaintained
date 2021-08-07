package com.swervedrivespecialties.swervelib.math;

public class MathUtil {
    public static final double VERY_SMALL_NUMBER = 1e-9;

    public static boolean almostEquals(double a, double b) {
        return almostEquals(a, b, VERY_SMALL_NUMBER);
    }

    public static boolean almostEquals(double a, double b, double allowableDifference) {
        return Math.abs(a - b) < allowableDifference;
    }
}
