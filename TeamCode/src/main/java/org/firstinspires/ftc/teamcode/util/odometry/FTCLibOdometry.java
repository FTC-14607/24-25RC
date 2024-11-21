package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

public class FTCLibOdometry {

    private MotorEx encoderLeft, encoderRight, encoderPerp;

    // ALL IN INCHES
    private double TRACK_WIDTH;         // distance between encoderRight and encoderLeft
    private double CENTER_WHEEL_OFFSET; // distance from encoderPerp to center of rotation
    private double TICKS_TO_INCHES;     // encoder ticks to inches

    private HolonomicOdometry holOdom;
    private OdometrySubsystem odometry;

    public FTCLibOdometry(MotorEx encoderLeft, MotorEx encoderRight, MotorEx encoderPerp,
                          double trackWidth, double centerWheelOffset, double ticksToInches
                          ){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderPerp = encoderPerp;
        this.TRACK_WIDTH = trackWidth;
        this.TICKS_TO_INCHES = ticksToInches;
        this.CENTER_WHEEL_OFFSET = centerWheelOffset;
    }

    public void init(){
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);
    }

    public void reset() {
        this.init();
    }
    public void update(){
        odometry.update();
    }
    public Pose2d getPose(){
        return odometry.getPose();
    }

}
