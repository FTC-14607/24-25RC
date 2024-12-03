package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.DriveTrain;

/**
 * Drives a mecanum drivetrain robot to given positions using a 3-wheel odometry localizer.
 */
public class MecanumThreeWheelOdometryDriver implements OdometryDriver {

    // connection to hardware
    private final RobotLocalizer odo;
    private final DriveTrain driveTrain;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    // internal states
    private DriveMode driveMode = DriveMode.DRIVE_TO_TARGET;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double holdTargetDuration = 1; // seconds
    public boolean showTelemetry = true;

    // controllers
    private PIDController translationController; // takes inches, outputs power
    private PIDController yawController; // takes radians, outputs power

    public ElapsedTime holdTimer = new ElapsedTime();

    public MecanumThreeWheelOdometryDriver(
            RobotLocalizer odo, DriveTrain driveTrain, LinearOpMode opMode,
            PIDController translationController, PIDController yawController
    ) {
        this.odo = odo;
        this.driveTrain = driveTrain;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.translationController = translationController;
        this.yawController = yawController;
    }

    public Pose2d getCurrentPose() { return currentPose; }

    public void resetPose() {
        odo.reset();
    }

    public void updatePose() {
        odo.update();
        currentPose = odo.getPose();
    }

    public void setControllers(PIDController trans, PIDController yaw) {
        translationController = trans;
        yawController = yaw;
    }

    public void setHoldTargetDuration(double duration) { holdTargetDuration = duration; }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public void setTarget(Pose2d target) {
        targetPose = target;

        /*
        setPoint is 0, and we deem that the target. Switching targets just means switching how you
        calculate distance to the target.
         */
        translationController.setSetPoint(0);
        yawController.setSetPoint(target.getHeading());
    }

    /**
     *
     * @param straightChange [inches]
     * @param lateralChange [inches]
     * @param yawChange [degrees]
     */
    public void setRelativeTarget(double straightChange, double lateralChange, double yawChange) {
        double yawCurrent = currentPose.getHeading();
        double xChange = straightChange * Math.cos(yawCurrent) + lateralChange * Math.sin(yawCurrent);
        double yChange = straightChange *-Math.sin(yawCurrent) + lateralChange * Math.cos(yawCurrent);

        double xTarget = currentPose.getX() + xChange;
        double yTarget = currentPose.getY() + yChange;
        double yawTarget = yawCurrent + Math.toRadians(yawChange);

        setTarget(new Pose2d(xTarget, yTarget, new Rotation2d(yawTarget)));
    }

    public boolean drive() {
        // in the future we could acoomodate splining and speed control
        return lineDrive();
    }

    /**
     * Drives to targetPose if it exists and driveMode is DRIVE_TO_TARGET. Should be called after
     * setTarget, and should be called every OpMode loop (while(opModeIsActive()) iteration.
     * @return true if the driver does not drive the motors (either because it is not supposed to
     * or if it has reached the target), false otherwise.
     */
    public boolean lineDrive() {
        if (driveMode == DriveMode.MANUAL || targetPose == null)
            return true;

        double xTarget = targetPose.getX();
        double yTarget = targetPose.getY();
        double yawTarget = targetPose.getHeading();
        boolean yawTargetIsPositive = yawTarget > 0;

        // read current values
        updatePose();
        double xCurrent   = currentPose.getX();
        double yCurrent   = currentPose.getY();
        double yawCurrent = currentPose.getHeading();

        // rebound yaw to either [0,360] or [-360,0] if crossing the 180/-180 degree mark
        double yawCurrentRebounded = yawCurrent;
        if      ( yawTargetIsPositive && yawCurrent < yawTarget - Math.PI) yawCurrentRebounded += 2*Math.PI;
        else if (!yawTargetIsPositive && yawCurrent > yawTarget + Math.PI) yawCurrentRebounded -= 2*Math.PI;

        // calculate errors
        double xError = xTarget - xCurrent;
        double yError = yTarget - yCurrent;
        double distanceToTarget = Math.hypot(xError, yError);

        /*
        calculate translational and rotational power. translationPower is negated because the
        PID has a setPoint of 0 and distanceToTarget is always positive (it's a distance), so
        the error will always be negative and thus the output would've been negative.
         */
        double translationPower = -translationController.calculate(distanceToTarget);
        double rotatePower      =  yawController        .calculate(yawCurrentRebounded);

        // angle between the x-axis and the displacement error vector
        double theta = Math.atan2(yError, xError);

        // angle between robot's heading and the displacement error vector
        double phi = yawCurrent - theta;

        /*
        Calculate throttle and strafe power. strafePower is negated because sine component
        points away from the yaw error, so it is negated to make the robot strafe towards the
        target rather than away.
         */
        double throttlePower = translationPower *  Math.cos(phi);
        double strafePower   = translationPower * -Math.sin(phi);

        // drive the robot while the target isn't reached
        if ( !translationController.atSetPoint() || !yawController.atSetPoint() ) {
            driveTrain.drive(throttlePower, strafePower, rotatePower);
            holdTimer.reset();
        }

        // otherwise targets are reached; hold position
        else {
            telemetry.addLine("Holding...");
            driveTrain.brake();
            if (holdTimer.time() > holdTargetDuration)
                return true;
        }

        // telemetry
        if (showTelemetry) {
            double yawError = Math.toDegrees(yawTarget - yawCurrent);
            telemetry.addData("Target (x,y,yaw)",
                    "(%5.2f, %5.2f, %6.2f)", xTarget, yTarget, Math.toDegrees(yawTarget));
            telemetry.addData("Current Pose (x,y,yaw)",
                    "(%5.2f, %5.2f, %6.2f)", xCurrent, yCurrent, Math.toDegrees(yawCurrent));
            telemetry.addData("Error (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", xError, yError, yawError);
            telemetry.addData("Distance to Target", distanceToTarget);
        }

        return false;
    }

    /**
     *
     * @param straightChange [inches]
     * @param lateralChange [inches]
     * @param yawChange [degrees
     */
    public void lineBy(double straightChange, double lateralChange, double yawChange) {
        updatePose();
        setRelativeTarget(straightChange, lateralChange, yawChange);

        while(opMode.opModeIsActive() && !opMode.isStopRequested())
            if (lineDrive())
                return;
    }

    /**
     *
     * @param x inches
     * @param y inches
     * @param yaw degrees
     */
    public void lineTo(double x, double y, double yaw) {
        setTarget(new Pose2d(x, y, new Rotation2d(Math.toRadians(yaw))));

        while(opMode.opModeIsActive() && !opMode.isStopRequested())
            if (lineDrive())
                return;
    }

}
