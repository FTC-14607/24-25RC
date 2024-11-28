package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

/**
 * Three-wheel odometry based localizer. Wraps FTCLib's odometry classes.
 */
public class FTCLibThreeWheelOdometry implements RobotLocalizer {

    private MotorEncoder encoderLeft, encoderRight, encoderPerp;

    private final double TRACK_WIDTH;         // [inches] distance between encoderRight and encoderLeft
    private final double CENTER_WHEEL_OFFSET; // [inches] distance from encoderPerp to center of rotation
    private final double TICKS_PER_ROTATION;  // encoder ticks
    private final double WHEEL_DIAMETER;      // [inches] radius of odometry wheels

    private double inchesPerTick;

    private HolonomicOdometry holOdom;
    private OdometrySubsystem odometry;

    /**
     * Connects localizer to encoder hardware. Constructor should be called after encoders are
     * initialized (mapped to using the hardwareMap).
     * @param encoderLeft
     * @param encoderRight
     * @param encoderPerp
     * @param trackWidth distance between encoderLeft and encoderRight
     * @param centerWheelOffset distance between perpendicular/horizontal encoder and center of rotation
     * @param ticksPerRotation encoder ticks per rotation
     * @param wheelDiameter diameter of odometry wheels
     */
    public FTCLibThreeWheelOdometry(
            MotorEncoder encoderLeft, MotorEncoder encoderRight, MotorEncoder encoderPerp,
            double trackWidth, double centerWheelOffset, double ticksPerRotation, double wheelDiameter
    ) {
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderPerp = encoderPerp;
        this.TRACK_WIDTH = trackWidth;
        this.TICKS_PER_ROTATION = ticksPerRotation;
        this.CENTER_WHEEL_OFFSET = centerWheelOffset;
        this.WHEEL_DIAMETER = wheelDiameter;

        inchesPerTick = Math.PI * WHEEL_DIAMETER / TICKS_PER_ROTATION;

        holOdom = new HolonomicOdometry(
                () -> encoderTicksToInches(encoderLeft.getCurrentPosition()),
                () -> encoderTicksToInches(encoderRight.getCurrentPosition()),
                () -> encoderTicksToInches(encoderPerp.getCurrentPosition()),
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);
    }

    private double encoderTicksToInches(int ticks) {
        return ticks * inchesPerTick;
    }

    public void reset() {
        encoderLeft.resetPosition();
        encoderRight.resetPosition();
        encoderPerp.resetPosition();
        update();
    }

    public void update(){
        odometry.update();
    }

    public Pose2d getPose(){
        return odometry.getPose();
    }

    // TODO: all these vv
    public Pose2d getVelocity() {

        return null;
    }

    public Pose2d getAcceleration() {
        return null;
    }

}
