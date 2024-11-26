package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.odometry.FTCLibOdometry;

@Config
public class Outreach extends MecanumDrive {

    // actuators and sensors
    public MotorEx odoRight;
    public MotorEx odoLeft;
    public MotorEx odoPerp;
    public FTCLibOdometry odo;

    // controllers
    public static double p_head = 0.60, i_head = 0, d_head = 0;
    public static double tol_head = 2*Math.PI/360; // 1 degree
    public PIDController headingController; // takes radians, outputs power

    public static double p_trans = 0.05, i_trans = 0.04, d_trans = 0;
    public static double tol_trans = 0.25;
    public PIDController translationController; // takes inches, outputs power

    public ElapsedTime holdTimer = new ElapsedTime();

    // telemetry
    public enum DriveMode { MANUAL, DRIVE_TO_TARGET }
    private DriveMode driveMode = DriveMode.MANUAL;

    public Pose2d currentPose;
    public Pose2d targetPose = null;
    private double holdTargetDuration;


    // physical constants
    public final static RobotDimensions DIMENSIONS = new RobotDimensions(
            18, 18, 18, 9.6, 537.68984
    );
    public static double TRACK_WIDTH = 8.2087;        // 20.85 cm
    public static double CENTER_WHEEL_OFFSET = 0;   // 0 cm
    // GoBilda Odometry Pod; 2000 ticks/rev, 48 mm diameter; somehow pods are reversed so * -1
    public static double TICKS_TO_INCHES = 3.14159 * (4.8 / 2.54) / 2000 * -1;

    public Outreach(LinearOpMode opmode){
        super(opmode);

        dimensions = DIMENSIONS;
        odoRight = new MotorEx(hardwareMap, "frontLeft"); // encoders are plugged in next to motors
        odoLeft =  new MotorEx(hardwareMap, "backRight");
        odoPerp =  new MotorEx(hardwareMap, "frontRight");
        odo = new FTCLibOdometry(odoLeft, odoRight, odoPerp, TRACK_WIDTH, CENTER_WHEEL_OFFSET, TICKS_TO_INCHES);
        odo.init();

        imu.resetYaw();

        headingController = new PIDController(p_head, i_head, d_head);

        headingController.setTolerance(tol_head);   // degrees

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);
    }

    public void resetPose() {
        odo.reset();
        updatePose();
    }

    public void updatePose() {
        odo.update();
        currentPose = odo.getPose();
    }

    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }

//    public void forward(double distance, double holdTime) {
//        move(distance, 0, 0, holdTime);
//    }
//
//    public void strafe(double distance, double holdTime) {
//        move(0, distance, 0, holdTime);
//    }
//
//    public void rotate(double angle, double holdTime) {
//        move(0, 0, angle, holdTime);
//    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public void setTarget(Pose2d target, double holdTime) {
        setDisplacement(target.minus(currentPose), holdTime);
    }

    public void setDisplacement(Transform2d displacement, double holdTime) {

        double originalX = currentPose.getX();
        double originalY = currentPose.getY();
        double originalHeading = Math.toDegrees(currentPose.getHeading());

        double deltaX = displacement.getTranslation().getX();
        double deltaY = displacement.getTranslation().getY();
        double deltaHeading = displacement.getRotation().getDegrees();


    }

    /**
     * Drives to targetPose if it exists and driveMode is DRIVE_TO_TARGET. Should be called after
     * setTarget/setDisplacement, and should be called every OpMode loop (while(opModeIsActive()) iteration.
     */
    public void drive() {
        if (driveMode == DriveMode.MANUAL || targetPose == null)
            return;

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();


    }

    /**
     * Displaces the robot in a (hopefully) straight path
     * @param displacement
     */
    public void moveBy(Pose2d displacement, double holdTime) {

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);
        headingController = new PIDController(p_head, i_head, d_head);
        headingController.setTolerance(tol_head);

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
        headingController.reset();

//        translationController.setSetPoint(displacementDistance);
        translationController.setSetPoint(0);
        headingController.setSetPoint(deltaHeading);

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
            double rotatePower = headingController.calculate(rotatedBy);

            double theta = Math.atan2(errorY, errorX); // angle between the x-axis and the displacement vector

            double phi = theta - currentHeading; // angle between robot's current direction and the displacement vector
            if      (phi >  Math.PI) phi -= 2*Math.PI;
            else if (phi < -Math.PI) phi += 2*Math.PI;

            double throttlePower = translationPower *  Math.cos(phi);
            double strafePower   = translationPower * -Math.sin(phi);

            // drive the robot while targets aren't reached
            if ( !translationController.atSetPoint() || !headingController.atSetPoint() ) {

                drive(throttlePower, strafePower, rotatePower);

                holdTimer.reset();

                // otherwise targets are reached; hold position
            } else {
                telemetry.addLine("Holding...");

                brake();

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
