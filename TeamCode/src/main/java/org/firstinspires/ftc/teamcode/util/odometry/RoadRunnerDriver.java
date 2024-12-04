package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.roadrunner.drive.SampleMecanumDrive;

/**
 * Interfaces RoadRunner with my autonomous driving setup. I would say to just have the robot
 * classes extend RoadRunner's SampleMecanumDrive, but Java doesn't allow multiple inheritance and
 * I think it's better organizationally to keep RoadRunner stuff in the RoadRunner classes. You'd
 * probably want to just access the mecanumDrive variable and call stuff from that, but I have it
 * implement AutoDriver for homogeneity.
 */
public class RoadRunnerDriver implements AutoDriver {

    public SampleMecanumDrive mecanumDrive;

    @Override
    public Pose2d getCurrentPose() {
        return null;
    }

    @Override
    public void resetPose() {

    }

    @Override
    public void updatePose() {

    }

    @Override
    public void setControllers(PIDController translation, PIDController yaw) {

    }

    @Override
    public void setTolerances(double transTol, double yawTol) {

    }

    @Override
    public void setHoldTargetDuration(double duration) {

    }

    @Override
    public void setDriveMode(DriveMode mode) {

    }

    @Override
    public void setTarget(Pose2d target) {

    }

    @Override
    public void setRelativeTarget(double straight, double lateral, double yaw) {

    }

    @Override
    public boolean drive() {
        return false;
    }

    @Override
    public void lineBy(double straight, double lateral, double yaw) {

    }

    @Override
    public void lineTo(double x, double y, double yaw) {

    }
}
