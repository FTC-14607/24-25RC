package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.geometry.Pose2d;

// TODO
/**
 * Localizer using built-in encoders on a four-wheel drive mecanum drivetrain and the built-in IMU
 * on the control hub. In general, this localizer is very inaccurate due to wheel slippage unless
 * you severely limit the robot's speed. It's best to only use this for rotating. Depending on your
 * drivetrain motors, going straight is probably OK, but strafing is probably not.
 */
public class MecanumBuiltInOdometry implements RobotLocalizer {

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
