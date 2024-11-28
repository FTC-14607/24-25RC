package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.DriveTrain;

// TODO: test if this works then cleanup this and Outreach
/**
 * Drives a robot to given positions using a localizer (odometry). Right now it only accommodates
 * a mecanum wheel drivetrain (and because other odo setups are untested, three wheel odometry).
 */
public class OdometryDriver {

    private RobotLocalizer odo;
    private DriveTrain driveTrain;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    public boolean showTelemetry = true;

    public ElapsedTime holdTimer = new ElapsedTime();

    // controllers
    public static double p_yaw = 0.60, i_yaw = 0, d_yaw = 0;
    public static double tol_yaw = 2*Math.PI/360; // 1 degree
    public PIDController yawController; // takes radians, outputs power

    public static double p_trans = 0.05, i_trans = 0.04, d_trans = 0;
    public static double tol_trans = 0.25; // quarter inch
    public PIDController translationController; // takes inches, outputs power


    public enum DriveMode { MANUAL, DRIVE_TO_TARGET }
    private DriveMode driveMode = DriveMode.DRIVE_TO_TARGET;

    public Pose2d currentPose;
    public Pose2d targetPose;
    private double holdTargetDuration;

    public static double TRACK_WIDTH = 8.2087;        // 20.85 cm
    public static double CENTER_WHEEL_OFFSET = 0;   // 0 cm

    // GoBilda Odometry Pod; 2000 ticks/rotation, 48 mm diameter
    public static double ODO_TICKS_PER_ROTATION = 2000;
    public static double ODO_WHEEL_DIAMETER = 48.0 / 10 / 2.54;

    public OdometryDriver(RobotLocalizer odo, DriveTrain driveTrain, LinearOpMode opMode){

        this.odo = odo;
        this.driveTrain = driveTrain;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        yawController = new PIDController(p_yaw, i_yaw, d_yaw);
        yawController.setTolerance(tol_yaw);

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);
    }

    public void resetPose() {
        odo.reset();
    }

    public void updatePose() {
        odo.update();
        currentPose = odo.getPose();
    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public void setTarget(Pose2d target) {
        targetPose = target;

        // TODO: after tuning, PID constructors move to class constructor
        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);

        yawController = new PIDController(p_yaw, i_yaw, d_yaw);
        yawController.setTolerance(tol_yaw);

        translationController.reset();
        yawController.reset();

        /*
        setPoint is 0, and we deem that the target. Switching targets just means switching how you
        calculate distance to the target.
         */
        translationController.setSetPoint(0);
        yawController.setSetPoint(target.getHeading());
    }

    public void setDisplacement() {

    }

    /**
     * Drives to targetPose if it exists and driveMode is DRIVE_TO_TARGET. Should be called after
     * setTarget/setDisplacement, and should be called every OpMode loop (while(opModeIsActive()) iteration.
     * @param holdTime
     * @return true if the target is reached and held for the given duration, false otherwise.
     */
    public boolean driveToTarget(double holdTime) {
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
            if (holdTimer.time() > holdTime) //:3 ~~~nyaaa
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

    public void moveBy(double xChange, double yChange, double yawChange, double holdTime) {
        updatePose();
        double xTarget = currentPose.getX() + xChange;
        double yTarget = currentPose.getY() + yChange;
        double yawTarget = currentPose.getRotation().getDegrees() + yawChange;

        moveTo(xTarget, yTarget, yawTarget, holdTime);
    }

    public void moveTo(double x, double y, double yaw, double holdTime) {
        setTarget(new Pose2d(x, y, new Rotation2d(Math.toRadians(yaw))));

        while(opMode.opModeIsActive() && !opMode.isStopRequested())
            if (driveToTarget(holdTime))
                return;
    }

    public void moveBy(Pose2d displacement, double holdTime) {

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);
        yawController = new PIDController(p_yaw, i_yaw, d_yaw);
        yawController.setTolerance(tol_yaw);

        // read starting values
        // TODO: convert to Vector2d
        updatePose();
        double originalX = currentPose.getX();
        double originalY = currentPose.getY();
        double originalHeading = currentPose.getHeading(); // radians

        boolean originalHeadingIsPositive = originalHeading >= 0;

        double deltaX = displacement.getX();
        double deltaY = displacement.getY();
        double deltaHeading = displacement.getHeading();

        double displacementDistance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);

        translationController.reset();
        yawController.reset();

//        translationController.setSetPoint(displacementDistance);
        translationController.setSetPoint(0);
        yawController.setSetPoint(deltaHeading);

        while(opMode.opModeIsActive() && !opMode.isStopRequested()) {

            updatePose();
            double currentX = currentPose.getX();
            double currentY = currentPose.getY();
            double currentHeading = currentPose.getHeading();

            double currentHeadingRebounded = currentHeading;

            // * account for rotating over the 180/-180 point
            if ( originalHeadingIsPositive != (currentHeading >= 0) )
                if      ( originalHeadingIsPositive && currentHeading < -Math.PI/2) currentHeadingRebounded += 2*Math.PI;
                else if (!originalHeadingIsPositive && currentHeading >  Math.PI/2) currentHeadingRebounded -= 2*Math.PI;
            double rotatedBy = currentHeadingRebounded - originalHeading;

            double traveledX = currentX - originalX;
            double traveledY = currentY - originalY;

            // use the displacement error vector (distance and angle to target) to calculate drive and strafe power
            double errorX = deltaX - traveledX;
            double errorY = deltaY - traveledY;
            double distToTarget = Math.sqrt(errorX*errorX + errorY*errorY);

            double translationPower = -translationController.calculate(distToTarget);
            double rotatePower = yawController.calculate(rotatedBy);

            double theta = Math.atan2(errorY, errorX); // angle between the x-axis and the displacement vector

            double phi = theta - currentHeading; // angle between robot's current direction and the displacement vector
            if      (phi >  Math.PI) phi -= 2*Math.PI;
            else if (phi < -Math.PI) phi += 2*Math.PI;

            double throttlePower = translationPower *  Math.cos(phi);
            double strafePower   = translationPower * -Math.sin(phi);

            // drive the robot while targets aren't reached
            if ( !translationController.atSetPoint() || !yawController.atSetPoint() ) {

                driveTrain.drive(throttlePower, strafePower, rotatePower);

                holdTimer.reset();

                // otherwise targets are reached; hold position
            } else {
                telemetry.addLine("Holding...");

                driveTrain.brake();

                if (holdTimer.time() > holdTime)
                    break;
            }

            telemetry.addData("Displacing By (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", deltaX, deltaY, Math.toDegrees(deltaHeading));
            telemetry.addData("Current Pose (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees());
            telemetry.addData("Errors (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", errorX, errorY, Math.toDegrees(deltaHeading - rotatedBy));
            telemetry.addData("Distance to Target", distToTarget);
            telemetry.update();
        }

    }


}
