package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.odometry.FTCLibThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.odometry.MecanumThreeWheelOdometryDriver;
import org.firstinspires.ftc.teamcode.util.odometry.MotorEncoder;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryDriver;

@Config
public class Outreach extends MecanumDrive {

    // actuators and sensors
    public MotorEncoder odoRight;
    public MotorEncoder odoLeft;
    public MotorEncoder odoPerp;

    // controllers
    public static double p_yaw = 0.60, i_yaw = 0, d_yaw = 0;
    public static double tol_yaw = 2*Math.PI/360; // 1 degree
    public PIDController yawController; // takes radians, outputs power

    public static double p_trans = 0.05, i_trans = 0.04, d_trans = 0;
    public static double tol_trans = 0.25; // quarter inch
    public PIDController translationController; // takes inches, outputs power
    public FTCLibThreeWheelOdometry odo;
    public OdometryDriver odoDriver;

    // physical constants
    public final static double TRACK_WIDTH = 8.2087;        // 20.85 cm
    public final static double CENTER_WHEEL_OFFSET = 0;   // 0 cm

    // GoBilda Odometry Pod; 2000 ticks/rotation, 48 mm diameter
    public final static double ODO_TICKS_PER_ROTATION = 2000;
    public final static double ODO_WHEEL_DIAMETER = 48.0 / 10 / 2.54;

    public Outreach(LinearOpMode opmode){
        super(opmode);

        // parent class constants (GoBilda 5203 312 RPM)
        ROBOT_HEIGHT = 18;
        ROBOT_LENGTH = 18;
        ROBOT_WIDTH = 18;
        WHEEL_DIAMETER = 9.6 / 2.54;
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        DRIVETRAIN_TICKS = 537.68984;
        STRAFE_MULTIPLIER = 1.3;
        STRAIGHT_ROTATION_CORRECTION = 0;
        STRAFE_ROTATION_CORRECTION = 0;

        // hardwareMap
        odoRight = new MotorEncoder(frontLeft); // encoders are plugged in next to motors
        odoLeft =  new MotorEncoder(backRight);
        odoPerp =  new MotorEncoder(frontRight);

        odoLeft.setDirection(MotorEncoder.Direction.REVERSE);
//        odoPerp.setDirection(MotorEncoder.Direction.REVERSE);

        odo = new FTCLibThreeWheelOdometry(
                odoLeft, odoRight, odoPerp,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET, ODO_TICKS_PER_ROTATION, ODO_WHEEL_DIAMETER);

        imu.resetYaw();

        yawController = new PIDController(p_yaw, i_yaw, d_yaw);
        yawController.setTolerance(tol_yaw);

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);

        odoDriver = new MecanumThreeWheelOdometryDriver(odo, this, opMode, translationController, yawController);
    }

    public Pose2d getCurrentPose() { return odoDriver.getCurrentPose(); }

    public void setOdoDriverTarget(Pose2d target) {
        // TODO: after tuning, this remove PID constructors.
        yawController = new PIDController(p_yaw, i_yaw, d_yaw);
        yawController.setTolerance(tol_yaw);

        translationController = new PIDController(p_trans, i_trans, d_trans);
        translationController.setTolerance(tol_trans);

        odoDriver.setControllers(translationController, yawController);
        odoDriver.setTarget(target);
    }

    public void addPoseToTelemetry() {
        telemetry.addData("Pose (x,y,heading)",
                "(%5.2f, %5.2f, %6.2f)",
                getCurrentPose().getX(), getCurrentPose().getY(), getCurrentPose().getRotation().getDegrees());
    }

}
