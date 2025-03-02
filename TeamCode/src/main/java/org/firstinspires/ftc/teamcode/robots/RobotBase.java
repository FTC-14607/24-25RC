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
 * any FTC Autonomous or Driver-Controlled purposes. A good utilization of Java's OOP organization
 * is to make each child class only implement what is unique to it. For example, you should not
 * re-implement mecanum drive code or odo code since those are common to many robots.
 */
public abstract class RobotBase {
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public VoltageSensor controlHubVoltageSensor;
    public IMU imu;

    public double controlHubVoltage;
    public YawPitchRollAngles orientation;

    public boolean showTelemetry = true;
    public double voltageScaler;

    public static final double NORMAL_VOLTAGE = 12.0; // volts

    // physical constants (would be better if these were final but that's annoying)
    public double ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT;

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
        voltageScaler = NORMAL_VOLTAGE / controlHubVoltage;
    }

    /**
     * Decreases power to emulate behavior when the robot is using an exactly 12V battery. Typically
     * healthy batteries charge to 14V, so to keep behavior consistent power should be scaled. Note
     * this uses the voltage read upon initialization, and does not re-read hub voltage. If voltage
     * is under 12 volts, no changes are made.
     * @param motorPower [0, 1]
     * @return
     */
    public double scalePower(double motorPower) {
        return motorPower * voltageScaler;
    }

    /**
     * Sets all the motors in motors[] to the passed run mode
     * @param motors drivetrain or slides
     * @param mode DcMotorEx.RunMode
     */
    public static void setRunMode(DcMotor[] motors, DcMotor.RunMode mode) {
        for (DcMotor motor : motors)
            motor.setMode(mode);
    }

    public static void setZeroPowerBehavior(DcMotor[] motors, DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor motor : motors)
            motor.setZeroPowerBehavior(behavior);
    }

    public static void setDirection(DcMotor[] motors, DcMotorSimple.Direction direction) {
        for (DcMotor motor : motors)
            motor.setDirection(direction);
    }

    public static double centimetersToInches(double centimeters) { return centimeters / 2.54; }
    public static double inchesToCentimeters(double inches) { return inches * 2.54; }

    public static boolean isClose(int a, int b, int tol) {
        return Math.abs(a - b) < tol;
    }

    public static boolean isClose(double a, double b, double tol) {
        return Math.abs(a - b) < tol;
    }

    /**
     * Range.clip except high/low bound order doesn't matter
     * @param num
     * @param bound1
     * @param bound2
     * @return
     */
    public static double clip(double num, double bound1, double bound2) {
        double low = bound1, high = bound2;
        if (bound1 > bound2) { low = bound2; high = bound1; }
        return (num < low) ? low : Math.min(num, high);
    }

    public static int clip(int num, int bound1, int bound2) {
        int low = bound1, high = bound2;
        if (bound1 > bound2) { low = bound2; high = bound1; }
        return (num < low) ? low : Math.min(num, high);
    }
    // generally using floats, shorts, or bytes won't make much of a performance difference and I'm too lazy to add them

    public static float clip(float num, float bound1, float bound2) {
        float high, low;
        if (bound1 < bound2) { low = bound1; high = bound2; }
        else                 { low = bound2; high = bound2; }
        return (num < low) ? low : Math.min(num, high);
    }

}