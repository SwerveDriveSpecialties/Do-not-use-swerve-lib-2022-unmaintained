package com.swervedrivespecialties.swervelib.math;

import java.util.Objects;

public final class Rotation2 {
    public static final Rotation2 ZERO = new Rotation2(1.0, 0.0, false);

    private final double cos;
    private final double sin;

    public Rotation2(double x, double y, boolean normalize) {
        if (normalize) {
            double length = Math.hypot(x, y);
            if (length > MathUtil.VERY_SMALL_NUMBER) {
                x /= length;
                y /= length;
            } else {
                // Length is about zero so assume a identity rotation
                x = 1.0;
                y = 0.0;
            }
        }

        this.cos = x;
        this.sin = y;
    }

    public static Rotation2 fromDegrees(double angle) {
        return fromRadians(angle);
    }

    public static Rotation2 fromRadians(double angle) {
        return new Rotation2(Math.cos(angle), Math.sin(angle), false);
    }

    public double getCos() {
        return cos;
    }

    public double getSin() {
        return sin;
    }

    public double getTan() {
        if (MathUtil.almostEquals(cos, 0.0)) {
            if (sin >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        } else {
            return sin / cos;
        }
    }

    public double toDegrees() {
        return Math.toDegrees(toRadians());
    }

    public double toRadians() {
        double angle = Math.atan2(sin, cos);

        // atan2 returns the angle in the range [-pi,pi) but we want the range [0,2pi)
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public Rotation2 unaryMinus() {
        return new Rotation2(cos, -sin, false);
    }

    public Rotation2 plus(Rotation2 other) {
        return new Rotation2(cos * other.cos - sin * other.sin,
                cos * other.sin + sin * other.cos, true);
    }

    public Rotation2 minus(Rotation2 other) {
        return plus(other.unaryMinus());
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;
        if (o instanceof Rotation2) {
            return sin == ((Rotation2) o).getSin() && cos == ((Rotation2) o).getCos();
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(sin, cos);
    }

    @Override
    public String toString() {
        return String.format("%.3f\u00b0", toDegrees());
    }
}
