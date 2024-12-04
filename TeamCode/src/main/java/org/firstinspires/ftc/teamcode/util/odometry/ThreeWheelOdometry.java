package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

/**
 * Three-wheel odometry based localizer. Wraps FTCLib's odometry classes.
 */
public class ThreeWheelOdometry implements RobotLocalizer {

    private final MotorEncoder encoderLeft, encoderRight, encoderPerp;
    private final double inchesPerTick;

    private final OdometrySubsystem odometry;

    /**
     * Connects localizer to encoder hardware. Constructor should be called after encoders are
     * initialized (mapped to using the hardwareMap).
     * @param encoderLeft encoder on the left
     * @param encoderRight encoder on the right
     * @param encoderPerp encoder that is perpendicular to the others
     * @param trackWidth [inches] distance between encoderLeft and encoderRight
     * @param centerWheelOffset [inches] distance between perpendicular/horizontal encoder and center of rotation
     * @param ticksPerRotation [ticks] encoder ticks per rotation
     * @param wheelDiameter [inches] diameter of odometry wheels
     */
    public ThreeWheelOdometry(
            MotorEncoder encoderLeft, MotorEncoder encoderRight, MotorEncoder encoderPerp,
            double trackWidth, double centerWheelOffset, double ticksPerRotation, double wheelDiameter
    ) {
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderPerp = encoderPerp;

        inchesPerTick = Math.PI * wheelDiameter / ticksPerRotation;

        odometry = new OdometrySubsystem(new HolonomicOdometry(
                () -> encoderTicksToInches(encoderLeft.getCurrentPosition()),
                () -> encoderTicksToInches(encoderRight.getCurrentPosition()),
                () -> encoderTicksToInches(encoderPerp.getCurrentPosition()),
                trackWidth, centerWheelOffset
        ));
    }

    private double encoderTicksToInches(int ticks) {
        return ticks * inchesPerTick;
    }

    @Override
    public void reset() {
        encoderLeft.resetPosition();
        encoderRight.resetPosition();
        encoderPerp.resetPosition();
        update();
    }

    @Override
    public void update(){
        odometry.update();
    }

    @Override
    public Pose2d getPose(){
        return odometry.getPose();
    }

    // TODO: all these vv
    @Override
    public Pose2d getVelocity() {

        return null;
    }

    @Override
    public Pose2d getAcceleration() {
        return null;
    }

}
