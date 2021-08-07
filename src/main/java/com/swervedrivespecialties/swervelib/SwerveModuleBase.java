package com.swervedrivespecialties.swervelib;

@Deprecated
public abstract class SwerveModuleBase implements SwerveModule {
    protected abstract void update(double drivePercentOutput, double targetSteerAngle);

    @Override
    public void set(double drivePercentOutput, double targetSteerAngle) {
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        double difference = targetSteerAngle - getSteerAngle();
        if (difference >= Math.PI) {
            targetSteerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            targetSteerAngle += 2.0 * Math.PI;
        }

        // If the difference is greater than 90 deg or less than -90 deg the drive output can be inverted so the
        // total movement of the module is less than 90 deg
        difference = targetSteerAngle - getSteerAngle();
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            targetSteerAngle += Math.PI;
            drivePercentOutput *= -1.0;
        }

        // Put target angle back into the range [0, 2pi)
        targetSteerAngle %= 2.0 * Math.PI;
        if (targetSteerAngle < 0.0) {
            targetSteerAngle += 2.0 * Math.PI;
        }

        update(drivePercentOutput, targetSteerAngle);
    }
}
