package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    public static double tP = 0.03, tI = 0.1, tD = 0, tF = 0;
    public static double sP = 0, sI = 0, sD = 0, sF = 0;
    public static double rP = 0, rI = 0, rD = 0, rF = 0;
    public PIDFController throttleController;
    public PIDFController strafeController;
    public PIDFController rotateController;
    public ElapsedTime holdTimer = new ElapsedTime();

    // telemetry
    public static Pose2d currentPose;

    // physical constants
    public final static RobotDimensions DIMENSIONS = new RobotDimensions(
            18, 18, 18, 9.6, 537.68984
    );
    public final static double TRACK_WIDTH = 8.72;        // 20.15 cm
    public final static double CENTER_WHEEL_OFFSET = 0;   // 0 cm
    // GoBilda Odometry Pod; 2000 ticks/rev, 48 mm diameter; somehow pods are reversed so * -1
    public final static double TICKS_TO_INCHES = 3.14159 * (4.8 / 2.54) / 2000 * -1;

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
        rotateController   = new PIDFController(rP, rI, rD, rF);

        throttleController.setTolerance(0.1); // inches
        strafeController  .setTolerance(0.5); // inches
        rotateController  .setTolerance(5);   // degrees
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

    public void move(double straightDistance, double strafeDistance, double rotateAngle, double holdTime) { // inches
        // read starting values
        updatePose();
        Pose2d firstPose = currentPose;

        // set controller setpoints
        throttleController.reset();
        strafeController  .reset();
        rotateController  .reset();

        throttleController.setSetPoint(straightDistance);
        strafeController  .setSetPoint(strafeDistance);
        rotateController  .setSetPoint(rotateAngle);

        while(opMode.opModeIsActive() && !opMode.isStopRequested()) {
            updatePose();

            double throttlePower = throttleController.calculate(currentPose.getX() - firstPose.getX());
            double strafePower   = strafeController  .calculate(currentPose.getY() - firstPose.getY());
            double rotatePower   = rotateController  .calculate(currentPose.getHeading() - firstPose.getHeading());

            drive(throttlePower, strafePower, rotatePower);

            // break loop if target is reached and it has been held long enough
            if (
                throttleController.atSetPoint() &&
                strafeController  .atSetPoint() &&
                rotateController  .atSetPoint()
            ) {
                telemetry.addLine("Holding...");
                stop();
                if (holdTimer.time() > holdTime)
                    break;
            } else
                holdTimer.reset();

            telemetry.addData("Now Pose", currentPose);
            telemetry.addData("First Pose", firstPose);
            telemetry.addData("Set Point", throttleController.getSetPoint());
            telemetry.addData("Current Point", currentPose.getX());
            telemetry.update();
        }

    }

}
