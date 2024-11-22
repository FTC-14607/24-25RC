package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Robot with nothing but a Control Hub. Intended to be extended by all hardware control classes for
 * any FTC Autonomous or Driver-Controlled purposes.
 */
public abstract class RobotBase {
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public VoltageSensor controlHubVoltageSensor;
    public IMU imu;

    public double controlHubVoltage;
    public YawPitchRollAngles orientation;
//    public BNO055IMU imu;

    public boolean showTelemetry = true;

    public static class RobotDimensions {
        // INCHES
        public final double ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT;
        // CENTIMETERS
        public final double WHEEL_DIAMETER, WHEEL_CIRCUMFERENCE;
        // TICKS / ROTATION (use -1 if built-in motor encoders are not used)
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

        // sensors
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");

        controlHubVoltage = controlHubVoltageSensor.getVoltage();
    }

    public RobotDimensions getDimensions() { return dimensions; }

    /**
     * Sets all the motors in motors[] to the passed run mode
     * @param motors drivetrain or slides
     * @param mode DcMotorEx.RunMode
     */
    public void setRunMode(DcMotor[] motors, DcMotor.RunMode mode) {
        for (DcMotor motor : motors)
            motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor[] motors, DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor motor : motors)
            motor.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotor[] motors, DcMotorSimple.Direction direction) {
        for (DcMotor motor : motors)
            motor.setDirection(direction);
    }
}