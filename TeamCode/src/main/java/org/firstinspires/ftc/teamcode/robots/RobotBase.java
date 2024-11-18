package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

/**
 * RobotBase is to be extended by all hardware control classes for any FTC Autonomous or
 * Driver-Controlled purposes
 */
public abstract class RobotBase {
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public IMU imu;
    public Orientation orientation;

    public static class RobotDimensions {
        // INCHES
        public final double ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT;
        // CENTIMETERS
        public final double WHEEL_DIAMETER, WHEEL_CIRCUMFERENCE;
        // (encoder resolution)
        public final double DRIVETRAIN_TICKS;

        public RobotDimensions(double robotLength, double robotWidth, double robotHeight, double wheelDiameter, double drivetrainTicks) {
            ROBOT_LENGTH = robotLength;
            ROBOT_WIDTH = robotWidth;
            ROBOT_HEIGHT = robotHeight;
            WHEEL_DIAMETER = wheelDiameter;
            WHEEL_CIRCUMFERENCE = Math.PI * wheelDiameter;
            DRIVETRAIN_TICKS = drivetrainTicks;
        }
    }

    protected static RobotDimensions dimensions;

    public RobotBase(LinearOpMode opModeInstance) {
        opMode = opModeInstance;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        // set bulk reads to auto
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public RobotDimensions getDimensions() { return dimensions; }

}