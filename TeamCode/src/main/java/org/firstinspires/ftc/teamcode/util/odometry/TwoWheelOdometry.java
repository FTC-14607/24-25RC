package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.geometry.Pose2d;

// TODO

/**
 * Localizer using two perpendicular odometry pods and the built-in IMU on the control hub. This
 * is supposedly slower than ThreeWheelOdometry since odo pods can be read faster than the IMU.
 */
public class TwoWheelOdometry implements RobotLocalizer {

    @Override
    public void reset() {

    }

    @Override
    public void update() {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public Pose2d getVelocity() {
        return null;
    }

    @Override
    public Pose2d getAcceleration() {
        return null;
    }
}
