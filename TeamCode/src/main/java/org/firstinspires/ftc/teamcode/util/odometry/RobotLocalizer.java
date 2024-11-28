package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public interface RobotLocalizer {

    void reset();
    void update();
    Pose2d getPose();
    Pose2d getVelocity();
    Pose2d getAcceleration();

}
