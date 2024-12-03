package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * Drives a robot to given positions using a localizer.
 */
public interface OdometryDriver {

    enum DriveMode { MANUAL, DRIVE_TO_TARGET }

    Pose2d getCurrentPose();
    void resetPose();
    void updatePose();

    void setControllers(PIDController translation, PIDController yaw);
    void setHoldTargetDuration(double duration);
    void setDriveMode(DriveMode mode);
    void setTarget(Pose2d target);
    void setRelativeTarget(double straight, double lateral, double yaw);

    boolean drive();

    void lineBy(double straight, double lateral, double yaw);
    void lineTo(double x, double y, double yaw);

}
