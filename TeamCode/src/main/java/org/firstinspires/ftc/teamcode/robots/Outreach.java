package org.firstinspires.ftc.teamcode.robots;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.odometry.FTCLibOdometry;


public class Outreach extends MecanumDrive {

    MotorEx odoRight;
    MotorEx odoLeft;
    MotorEx odoMiddle;
    public FTCLibOdometry odo;

    public static double TRACK_WIDTH = 8.72; // 20.15 cm, dist between right and left odo
    public static double CENTER_WHEEL_OFFSET = 0; // cm
    // GoBilda Odometry Pod; 2000 ticks/rev, 48 mm diameter
    public static double TICKS_TO_INCHES = 3.14159 * (4.8 / 2.54) / 2000;


    public Outreach(LinearOpMode opmode){
        super(opmode);
        // Drivetrain Motors: goBilda 5203 Series Yellow Jacket Planetary Gear Motor, 312 RPM
        dimensions = new RobotDimensions(-1, -1, -1, 4.8, 2000);

        // ODOMETRY
        odoRight = new MotorEx(hardwareMap, "frontLeft");
        odoLeft = new MotorEx(hardwareMap, "backRight");
        odoMiddle = new MotorEx(hardwareMap, "frontRight");

        odo = new FTCLibOdometry(odoLeft, odoRight, odoMiddle, TRACK_WIDTH, CENTER_WHEEL_OFFSET, TICKS_TO_INCHES);

    }

    public void forwardOdo(double distance) {

        PIDFController pid = new PIDFController(0.05, 0, 0, 0);
        pid.setTolerance(0.1);

        odo.update();
        double firstPos = odo.getPose().getY();

        pid.reset();
        pid.setSetPoint(distance);

        do {
            odo.update();
            double pos = odo.getPose().getX();
            double power = Range.clip(pid.calculate(pos - firstPos), -1, 1);

            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

        } while (opMode.opModeIsActive() && !opMode.isStopRequested() && !pid.atSetPoint());

        stop();

    }

    // ------------------------------------ INTERACTOR METHODS -------------------------------------


}
