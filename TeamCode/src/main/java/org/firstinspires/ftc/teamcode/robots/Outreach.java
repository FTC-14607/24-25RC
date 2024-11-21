package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.odometry.FTCLibOdometry;

@Config
public class Outreach extends MecanumDrive {

    // actuators and sensors
    public DcMotorEx vertSlideRight;
    public DcMotorEx vertSlideLeft;
    public Servo horiSlideRight; // axon servos
    public Servo horiSlideLeft;
    public MotorEx odoRight;
    public MotorEx odoLeft;
    public MotorEx odoPerp;
    public FTCLibOdometry odo;

    // controllers
    public static double tP = 0.025, tI = 0, tD = 0, tF = 0;
    public static double sP = 0, sI = 0, sD = 0, sF = 0;
    public static double rP = 0.01, rI = 0, rD = 0, rF = 0;
    public static double tTol = 0.1, sTol = 0.5, rTol = 5;
    public PIDFController throttleController; // takes inches
    public PIDFController strafeController; // takes inches
    public PIDFController headingController; // takes radians

    public static double trP = 0.025, trI = 0, trD = 0, trF = 0, trTol = 3;
    public PIDFController translationController; // takes inches, outputs power
    public ElapsedTime holdTimer = new ElapsedTime();

    // telemetry
    public static Pose2d currentPose;

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

        throttleController = new PIDFController(tP, tI, tD, tF);
        strafeController   = new PIDFController(sP, sI, sD, sF);
        headingController = new PIDFController(rP, rI, rD, rF);

        throttleController.setTolerance(tTol); // inches
        strafeController  .setTolerance(sTol); // inches
        headingController.setTolerance(rTol);   // degrees

        translationController = new PIDFController(trP, trI, trD, trF);
        translationController.setTolerance(trTol);
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

    public void forward(double distance, double holdTime) {
        move(distance, 0, 0, holdTime);
    }

    public void strafe(double distance, double holdTime) {
        move(0, distance, 0, holdTime);
    }

    public void rotate(double angle, double holdTime) {
        move(0, 0, angle, holdTime);
    }

    /**
     * Displaces the robot in a (hopefully) straight path
     * @param displacement
     */
    public void moveBy(Pose2d displacement, double holdTime) { // inches
        // read starting values
        updatePose();
        Pose2d firstPose = currentPose;
        double firstHeading = Math.toDegrees(firstPose.getHeading());
        boolean firstHeadingPositive = firstPose.getHeading() >= 0;

        double deltaX = displacement.getX();
        double deltaY = displacement.getY();

        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);

        // TODO: headings
        double deltaHeading = Math.toDegrees(displacement.getHeading()); // radians

        translationController.reset();
        headingController.reset();

        translationController.setSetPoint(distance);
        headingController.setSetPoint(deltaHeading);

        while(opMode.opModeIsActive() && !opMode.isStopRequested()) {

            updatePose();
            double currentHeading = Math.toDegrees(currentPose.getHeading());
            double currentHeadingRebounded = currentHeading;

            // * account for rotating over the 180/-180 point
            if ( firstHeadingPositive != (currentHeading >= 0) )
                if      ( firstHeadingPositive && currentHeading < -90) currentHeadingRebounded += 360;
                else if (!firstHeadingPositive && currentHeading >  90) currentHeadingRebounded -= 360;

            double traveledX = currentPose.getX() - firstPose.getX();
            double traveledY = currentPose.getY() - firstPose.getY();

            double distFromStart = Math.sqrt(traveledX*traveledX + traveledY*traveledY);

            double rotatedBy = currentHeadingRebounded - firstHeading;

            double translationPower = translationController.calculate(distFromStart);
            double rotatePower = headingController.calculate(rotatedBy);

            // use the displacement error vector (distance and angle to target) to calculate drive and strafe pwoer
            double errorX = displacement.getX() - currentPose.getX();
            double errorY = displacement.getY() - currentPose.getY();
            double distanceError = Math.sqrt(errorX*errorX + errorY*errorY);
            double theta = Math.atan2(errorY, errorX); // angle between the x-axis and the displacement vector

            double phi = theta - Math.toRadians(currentHeading); // angle between robot's current direction and the displacement vector
            if (phi > 180) phi -= 360;
            else if (phi < -180) phi += 360;


            double throttlePower = translationPower * Math.cos(phi);
            double strafePower = translationPower * Math.sin(phi);

            double headingPos  = currentHeading - Math.toDegrees(firstPose.getHeading());

            // drive the robot while targets aren't reached
            if ( !translationController.atSetPoint() || !headingController.atSetPoint() ) {

                drive(throttlePower, strafePower, rotatePower);

                holdTimer.reset();

                // otherwise targets are reached; hold position
            } else {
                brake();

                if (holdTimer.time() > holdTime)
                    break;

                telemetry.addLine("Holding...");
            }

            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Distance from start", distFromStart);
            telemetry.addData("Distance to end", distanceError);
            telemetry.addData("theta", theta);
            telemetry.addData("phi", phi);

        }

    }

    /**
     *
     * @param straightDistance inches
     * @param strafeDistance inches
     * @param rotateAngle degrees
     * @param holdTime seconds
     */
    public void move(double straightDistance, double strafeDistance, double rotateAngle, double holdTime) { // inches
        // read starting values
        updatePose();
        Pose2d firstPose = currentPose;
        Pose2d lastPose  = currentPose.plus(new Transform2d());
        boolean firstHeadingPositive = firstPose.getHeading() >= 0;

        // set controller setpoints
        throttleController.reset();
        strafeController  .reset();
        headingController.reset();

        throttleController.setSetPoint(straightDistance);
        strafeController  .setSetPoint(strafeDistance);
        headingController.setSetPoint(rotateAngle);
//        rotateController  .setSetPoint(Math.toRadians(rotateAngle));

        while(opMode.opModeIsActive() && !opMode.isStopRequested()) {

            // get current displacement from start
            updatePose();
            double currentHeading = Math.toDegrees(currentPose.getHeading());

            // * account for rotating over the 180/-180 point
            if ( firstHeadingPositive != (currentHeading >= 0) )
                if      ( firstHeadingPositive && currentHeading < -90) currentHeading += 360;
                else if (!firstHeadingPositive && currentHeading >  90) currentHeading -= 360;

            double headingPos  = currentHeading - Math.toDegrees(firstPose.getHeading());

            double errorX = currentPose.getX() - firstPose.getX();
            double errorY = currentPose.getY() - firstPose.getY();
            double distanceError = Math.sqrt(errorX*errorX + errorY*errorY);
            double theta = Math.atan2(errorY, errorX); // radians

            double phi = Math.toRadians(currentHeading) - theta;
            double straightError = distanceError * Math.cos(phi);
            double strafeError = distanceError * Math.sin(phi);

            double straightPos = straightDistance - straightError;
            double strafePos = strafeDistance - strafeError;

            // calculate controller outputs
            double throttlePower = throttleController.calculate(straightPos);
            double strafePower   = strafeController  .calculate(strafePos);
            double rotatePower   = headingController.calculate(headingPos);

            // drive the robot while targets aren't reached
            if (
                    !throttleController.atSetPoint() ||
                            !strafeController  .atSetPoint() ||
                            !headingController.atSetPoint()
            ) {
                drive(throttlePower, strafePower, rotatePower);

                holdTimer.reset();

                // otherwise targets are reached; hold position
            } else {
                brake();

                if (holdTimer.time() > holdTime)
                    break;

                telemetry.addLine("Holding...");
            }

            telemetry.addData("Target", "%5.2f | %5.2f | %5.2f",
                    straightDistance, strafeDistance, rotateAngle);
            telemetry.addData("Current Pose", currentPose);
//            telemetry.addData("Error (t,s,r)", "%5.2f | %5.2f | %5.2f",
//                    throttleError, strafeError, rotatePos);

            telemetry.addData("Straight Target", straightDistance);
            telemetry.addData("Straight Error", straightDistance - straightPos);
            telemetry.addData("Straight Power", throttlePower);
            telemetry.addData("Strafe Target", strafeDistance);
            telemetry.addData("Strafe Error", strafeDistance - strafePos);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Rotate Target", rotateAngle);
            telemetry.addData("Rotate Error", rotateAngle - headingPos);
            telemetry.addData("Rotate Power", rotatePower);
            telemetry.update();
        }

    }

}
