package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.DriveTrain;

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
    void setDisplacement(Pose2d displacement);
    boolean lineToTarget();
    void lineBy(double x, double y, double yaw);
    void lineTo(double x, double y, double yaw);

}
