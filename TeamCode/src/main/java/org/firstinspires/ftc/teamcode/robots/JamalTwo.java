package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.odometry.AutoDriver;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.odometry.MecanumThreeWheelOdometryDriver;
import org.firstinspires.ftc.teamcode.util.odometry.MotorEncoder;
import org.firstinspires.ftc.teamcode.util.odometry.RobotLocalizer;

@Config
public class JamalTwo extends MecanumDrive {

    // hardware
    // * drivetrain: goBILDA 96mm Mecanum Wheels and goBILDA 5203 312 RPM Motors
    public DcMotorEx    upperSlideRight; // goBILDA 5203 312 RPM
    public DcMotorEx    upperSlideLeft;  // same
    public Servo        upperArmRight;   // Axon Mini
    public Servo        upperArmLeft;    // same
    public Servo        upperClaw;       // goBILDA Torque
    public Servo        upperClawPitch;  // same

    public Servo        lowerSlideRight; // same
    public Servo        lowerSlideLeft;  // same
    public Servo        lowerClaw;       // same
    public Servo        lowerClawYaw;    // same
    public Servo        lowerClawPitch;  // same

    public MotorEncoder odoRight; // goBILDA Swingarm Odometry Pod (2000 ticks/rot, 48mm wheel)
    public MotorEncoder odoLeft;  // same
    public MotorEncoder odoPerp;  // same

    // control
    public DcMotorEx[]  upperSlides;
    public Servo[]      lowerSlides;

    public AutoDriver autoDriver;
    public RobotLocalizer odo;
    public PIDController translationController;
    public PIDController yawController;
    public static double trans_P = 0, trans_I = 0, trans_D = 0, trans_Tol = 0;
    public static double yaw_P = 0, yaw_I = 0, yaw_D = 0, yaw_Tol = 0;

    public static PIDFController vertSlidesController;
    public static double p_sli = 0, i_sli = 0, d_sli = 0;
    public static double f_sli = 0.3; // TODO: tune this

    public static PIDFCoefficients upperSlideVelocityPIDFCoefficients  = new PIDFCoefficients(1,0,0,0.5);
    public static PIDFCoefficients upperSlidesPositionPIDFCoefficients = new PIDFCoefficients(1,0,0,0); // only p matters for position

    // telemetry
    public static Pose2d currentPose;

    // constants
    public static double TRACK_WIDTH = 8.2087;    // 20.85 cm TODO
    public static double CENTER_WHEEL_OFFSET = 0; // 0 cm TODO
    public final static double ODO_TICKS_PER_ROTATION = 2000;
    public final static double ODO_WHEEL_DIAMETER = centimetersToInches(48. / 10);

    public static int UPPER_SLIDES_BOTTOM = 0; // ticks
    public static int UPPER_SLIDES_TOP = 4600;
    public static int UPPER_SLIDES_ABOVE_HIGH_CHAMBER = 3050;
    public static int UPPER_SLIDES_BELOW_HIGH_CHAMBER = 2075;
    public static int UPPER_SLIDES_PICKUP_SPECIMEN = 750;
    public static double VERT_SLIDES_DEFAULT_SPEED = 100;
    public static double VERT_SLIDES_INCHES_TO_TICKS = -1;

    public static double UPPER_ARM_LOWERED = 0; // servo position [0, 1]
    public static double UPPER_ARM_RAISED = 1;

    public static double UPPER_CLAW_CLOSED = 0.862;
    public static double UPPER_CLAW_OPEN = 0.735;
    public static double UPPER_CLAW_DOWN = 0;
    public static double UPPER_CLAW_UP = 1;
    public static double UPPER_CLAW_SCORE_SPECIMEN = 0.5;

    public static double LOWER_SLIDES_RETRACTED = 0.86;
    public static double LOWER_SLIDES_EXTENDED = 0.6;

    public static double LOWER_CLAW_CLOSED = 0.84;
    public static double LOWER_CLAW_OPEN = 0.64;
    public static double LOWER_CLAW_HORIZONTAL = 0.46; // to pick up vertical samples
    public static double LOWER_CLAW_VERTICAL = 0.14;
    public static double LOWER_CLAW_VERTICAL_FLIPPED = 0.78;
    public static double LOWER_CLAW_DOWN = 0.2;
    public static double LOWER_CLAW_UP = 1;


    public JamalTwo(LinearOpMode opmode) {
        super(opmode);

        // parent class constants
        ROBOT_HEIGHT = 18;
        ROBOT_LENGTH = 18;
        ROBOT_WIDTH = 18;
        WHEEL_DIAMETER = 9.6 / 2.54;
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        DRIVETRAIN_TICKS = 537.68984;
        STRAFE_MULTIPLIER = 1.3;
        STRAIGHT_ROTATION_CORRECTION = 0;
        STRAFE_ROTATION_CORRECTION = 0;

        // connect to hardware
        upperSlideRight = hardwareMap.get(DcMotorEx.class, "upperSlideRight");
        upperSlideLeft  = hardwareMap.get(DcMotorEx.class, "upperSlideLeft");
        upperArmRight   = hardwareMap.get(Servo.class,     "upperArmRight");
        upperArmLeft    = hardwareMap.get(Servo.class,     "upperArmLeft");
        upperClaw       = hardwareMap.get(Servo.class,     "upperClaw");
        upperClawPitch  = hardwareMap.get(Servo.class,     "upperClawPitch");

        lowerSlideRight = hardwareMap.get(Servo.class,     "lowerSlideRight");
        lowerSlideLeft  = hardwareMap.get(Servo.class,     "lowerSlideLeft");
        lowerClaw       = hardwareMap.get(Servo.class,     "lowerClaw");
        lowerClawYaw    = hardwareMap.get(Servo.class,     "lowerClawYaw");
        lowerClawPitch  = hardwareMap.get(Servo.class,     "lowerClawPitch");

        odoRight = new MotorEncoder(frontLeft); // encoders are plugged in next to motors
        odoLeft =  new MotorEncoder(backRight);
        odoPerp =  new MotorEncoder(frontRight);

        // initialization and setup
        upperSlides = new DcMotorEx[] {upperSlideRight, upperSlideLeft};
        lowerSlides = new Servo[] {lowerSlideRight, lowerSlideLeft};

        upperSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        upperArmRight.setDirection(Servo.Direction.REVERSE);
//        upperClawPitch.setDirection(Servo.Direction.REVERSE);
        lowerSlideRight.setDirection(Servo.Direction.REVERSE);

        odoLeft.setDirection(MotorEncoder.Direction.REVERSE);
//        odoPerp.setDirection(MotorEncoder.Direction.REVERSE);

        setZeroPowerBehavior(upperSlides, DcMotor.ZeroPowerBehavior.BRAKE); // TODO: experiment with this
//        setRunMode(vertSlides, DcMotor.RunMode.RUN_TO_POSITION);

        imu.resetYaw();

        odo = new ThreeWheelOdometry(
                odoLeft, odoRight, odoPerp,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET, ODO_TICKS_PER_ROTATION, ODO_WHEEL_DIAMETER
        );

        translationController = new PIDController(trans_P, trans_I, trans_D);
        translationController.setTolerance(trans_Tol);

        yawController = new PIDController(yaw_P, yaw_I, yaw_D);
        yawController.setTolerance(yaw_Tol);

        for (DcMotorEx slide : upperSlides) {
            slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, upperSlideVelocityPIDFCoefficients);
            slide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,   upperSlidesPositionPIDFCoefficients);
        }

        autoDriver = new MecanumThreeWheelOdometryDriver(odo, this, opMode, translationController, yawController);

    }

    // ---------------------------------------- SENSORS --------------------------------------------

    public void resetPose() {
        odo.reset();
        updatePose();
    }

    public void updatePose() {
        autoDriver.updatePose();
        currentPose = autoDriver.getCurrentPose();
    }

    public void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }

    // -------------------------------------- INTERACTORS ------------------------------------------

    public double getUpperArmPos()         { return upperArmRight.getPosition(); }
    public double getUpperClawPos()     { return upperClaw.getPosition(); }
    public double getUpperClawPitchPos()           { return upperClawPitch.getPosition(); }
    public double getLowerClawPos()       { return lowerClaw.getPosition(); }
    public double getLowerClawYawPos()    { return lowerClawYaw.getPosition(); }
    public double getLowerClawPitchPos()  { return lowerClawPitch.getPosition(); }

    // TODO: maybe use Range.scale
    public void setUpperClawPos(double pos) { upperClaw.setPosition(clip(pos, UPPER_CLAW_OPEN, UPPER_CLAW_CLOSED)); }
    public void setUpperClawPitchPos(double pos) { upperClawPitch.setPosition(clip(pos, UPPER_CLAW_DOWN, UPPER_CLAW_UP)); }
    public void setLowerClawPos(double pos) { lowerClaw.setPosition(clip(pos, LOWER_CLAW_CLOSED, LOWER_CLAW_OPEN)); }
    public void setLowerClawYawPos(double pos) { lowerClawYaw.setPosition(clip(pos, LOWER_CLAW_VERTICAL, LOWER_CLAW_VERTICAL_FLIPPED)); }
    public void setLowerClawPitchPos(double pos) { lowerClawPitch.setPosition(clip(pos, LOWER_CLAW_DOWN, LOWER_CLAW_UP)); }

    public void raiseArm()                { setUpperArmAndDumperPos(UPPER_ARM_RAISED); }
    public void lowerArm()                { setUpperArmAndDumperPos(UPPER_ARM_LOWERED); }
    public void raiseUpperClaw()              { setUpperClawPitchPos(UPPER_CLAW_SCORE_SPECIMEN); }
    public void lowerUpperClaw()             { setUpperClawPitchPos(UPPER_CLAW_DOWN); }
    public void extendLowerSlides()  { setHorizontalSlidesPos(LOWER_SLIDES_EXTENDED); }
    public void retractLowerSlides() { setHorizontalSlidesPos(LOWER_SLIDES_RETRACTED); }
    public void closeLowerClaw()         { setLowerClawPos(LOWER_CLAW_CLOSED); }
    public void openLowerClaw()          { setLowerClawPos(LOWER_CLAW_OPEN); }
    public void lowerLowerClaw()         { setLowerClawPitchPos(LOWER_CLAW_DOWN); }
    public void raiseLowerClaw()         { setLowerClawPitchPos(LOWER_CLAW_UP); }
    public void closeUpperClaw()       { setUpperClawPos(UPPER_CLAW_CLOSED); }
    public void openUpperClaw()        {  setUpperClawPos(UPPER_CLAW_OPEN); }

    public double getHorizontalSlidesPos() { return lowerSlideLeft.getPosition(); }

    public void setUpperArmPos(double pos) {
        upperArmRight.setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
        upperArmLeft.setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
    }


    public void setHorizontalSlidesPos(double pos) {
        pos = clip(pos, LOWER_SLIDES_EXTENDED, LOWER_SLIDES_RETRACTED);
        for (Servo slide : lowerSlides)
            slide.setPosition(pos);
        // TODO: add camera for auto pickup
    }

    public int getVerticalSlidePos() {
        // return vertSlideRight.getCurrentPosition();
        // Math.max since one motor may be broken, returning 0
        return Math.max(upperSlideLeft.getCurrentPosition(), upperSlideRight.getCurrentPosition());
        // TODO: test loop speed
    }

    /**
     * @param velocity [ticks/sec], positive is up
     */
    public void setVerticalSlidesVelocity(double velocity) { // ticks/s
        int currentPos = getVerticalSlidePos();

        // prevent exceeding slide limits
        if ( (currentPos >= UPPER_SLIDES_TOP && velocity > 0) || (currentPos <= UPPER_SLIDES_BOTTOM && velocity < 0) )
            return;

        // set velocity
        for (DcMotorEx slide : upperSlides) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setVelocity(velocity);
        }
    }


    /**
     * @param pos [ticks]
     */
    public void setVerticalSlidesPos(int pos) {
        // TODO: PIDF - find F, also use external PID
        int currentPos = getVerticalSlidePos();
        pos = Range.clip(pos, UPPER_SLIDES_BOTTOM, UPPER_SLIDES_TOP);

        for (DcMotorEx slide : upperSlides) {
            slide.setTargetPosition(pos);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // TODO: test this
            slide.setPower( (pos >= getVerticalSlidePos()) ? 0.8 : 0.5);
//            slide.setVelocity( (pos > currentPos) ? VERT_SLIDES_DEFAULT_SPEED : 0.8*VERT_SLIDES_DEFAULT_SPEED);
        }
    }

    /**
     * @param setPos [0-1]
     */
    public void setUpperArmAndDumperPos(double setPos) { // needs to keep dumper level while raising arm
        setPos = Range.clip(setPos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED);

        double currentPos = getUpperArmPos();
        double dumperPos = getUpperClawPitchPos();
        double posChange = setPos - currentPos;

        upperArmRight.setPosition(setPos);
        setUpperClawPitchPos(dumperPos - posChange);  // dumper rotates the opposite way by the same amount
    }

}
