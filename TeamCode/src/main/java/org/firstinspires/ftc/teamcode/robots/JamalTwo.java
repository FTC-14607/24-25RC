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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.odometry.AutoDriver;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.odometry.MecanumThreeWheelOdometryDriver;
import org.firstinspires.ftc.teamcode.util.odometry.MotorEncoder;
import org.firstinspires.ftc.teamcode.util.odometry.RobotLocalizer;

@Config
public class JamalTwo extends MecanumDrive {

    //region hardware
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
    //endregion
    //region control
    public DcMotorEx[] upperSlides;
    public int upperSlideRightPos, upperSlideLeftPos, upperSlidesPos;
    public double upperSlideRightVelo, upperSlideLeftVelo;
    public static PIDFController upperSlidesVelocityController;
    public static PIDController  upperSlidesPositionController;
    public double upperSlidesTargetVelocity;

    public Servo[] lowerSlides;

    public static Pose2d currentPose;
    public AutoDriver autoDriver;
    public RobotLocalizer odo;
    public PIDController translationController;
    public PIDController yawController;
    public static double trans_P = 0, trans_I = 0, trans_D = 0, trans_Tol = 0;
    public static double yaw_P = 0, yaw_I = 0, yaw_D = 0, yaw_Tol = 0;
    //endregion
    //region constants
    public static double TRACK_WIDTH = 8.2087;    // 20.85 cm TODO
    public static double CENTER_WHEEL_OFFSET = 0; // 0 cm TODO
    public final static double ODO_TICKS_PER_ROTATION = 2000;
    public final static double ODO_WHEEL_DIAMETER = centimetersToInches(48. / 10);

    public static int UPPER_SLIDES_BOTTOM = 0; // ticks
    public static int UPPER_SLIDES_TOP = 4600;
    public static int UPPER_SLIDES_ABOVE_HIGH_CHAMBER = 3050;
    public static int UPPER_SLIDES_BELOW_HIGH_CHAMBER = 2075;
    public static int UPPER_SLIDES_MIN_ARM_CLEARANCE = 500; // minimum height at which the upper arm can go fully down
    public static int UPPER_SLIDES_PICKUP_SPECIMEN = 750;
    public static int UPPER_SLIDES_DEPOSIT_SPECIMEN = 800;
    public static int UPPER_SLIDES_DEPOSIT_SAMPLE = 4000;

    public static double UPPER_SLIDES_DEFAULT_SPEED = 100;
    public static double UPPER_SLIDES_INCHES_TO_TICKS = -1;

    public static double UPPER_ARM_LOWERED = 0.1; // servo position [0, 1]
    public static double UPPER_ARM_RAISED = 1;
    public static double UPPER_ARM_PICKUP_SPECIMEN = 0.09;
    public static double UPPER_ARM_DEPOSIT_SPECIMEN = 0.5;
    public static double UPPER_ARM_DEPOSIT_SAMPLE = 1;

    public static double UPPER_CLAW_CLOSED = 0.862;
    public static double UPPER_CLAW_OPEN = 0.735;
    public static double UPPER_CLAW_DOWN = 0;
    public static double UPPER_CLAW_UP = 1;
    public static double UPPER_CLAW_PITCH_PICKUP_SPECIMEN = 0.91;
    public static double UPPER_CLAW_PITCH_DEPOSIT_SPECIMEN = 0.5;
    public static double UPPER_CLAW_PITCH_DEPOSIT_SAMPLE = 0.5;
    public static double UPPER_CLAW_PITCH_TRANSFER = 1;

    public static double LOWER_SLIDES_RETRACTED = 0.134;
    public static double LOWER_SLIDES_EXTENDED = 0.422;

    public static double LOWER_CLAW_CLOSED = 0.83;
    public static double LOWER_CLAW_OPEN = 0.55;
    public static double LOWER_CLAW_HORIZONTAL = 0.0574; // to pick up vertical samples
    public static double LOWER_CLAW_VERTICAL = 0.3786;
    public static double LOWER_CLAW_HORIZONTAL_FLIPPED = 0.708;
    public static double LOWER_CLAW_DOWN = 0.3984;
    public static double LOWER_CLAW_UP = 1;
    public static double LOWER_CLAW_PITCH_TRANSFER = 1;
    public static double LOWER_CLAW_YAW_TRANSFER = LOWER_CLAW_HORIZONTAL;
    //endregion

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
        upperSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        setZeroPowerBehavior(upperSlides, DcMotor.ZeroPowerBehavior.BRAKE); // TODO: experiment with this
        setRunMode(upperSlides, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upperArmRight.setDirection(Servo.Direction.REVERSE);

        // TODO: internal servo scaling
//        upperClaw.scaleRange(UPPER_CLAW_CLOSED, UPPER_CLAW_OPEN); //
//        upperClawPitch.setDirection(Servo.Direction.REVERSE);

        lowerSlides = new Servo[] {lowerSlideRight, lowerSlideLeft};
        lowerSlideRight.setDirection(Servo.Direction.REVERSE);

        odoLeft.setDirection(MotorEncoder.Direction.REVERSE);
//        odoPerp.setDirection(MotorEncoder.Direction.REVERSE);

        imu.resetYaw();

        odo = new ThreeWheelOdometry(
                odoLeft, odoRight, odoPerp,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET, ODO_TICKS_PER_ROTATION, ODO_WHEEL_DIAMETER
        );

        translationController = new PIDController(trans_P, trans_I, trans_D);
        translationController.setTolerance(trans_Tol);

        yawController = new PIDController(yaw_P, yaw_I, yaw_D);
        yawController.setTolerance(yaw_Tol);

        autoDriver = new MecanumThreeWheelOdometryDriver(odo, this, opMode, translationController, yawController);

    }

    // ------------------------------------ CONTROL & SENSORS ----------------------------------------

    /**
     * Re-reads all sensors and updates FSM loops
     */
    public void update() {
        updatePose();
        updateUpperSlides();
        updateSampleTransferFSM();
        updateSpecimenPickupFSM();
    }

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

    // ----------------------------------- INTERACTORS ---------------------------------------

    //region Upper Slides
    private enum UpperSlidesState { INACTIVE, RUN_BY_VELOCITY, RUN_TO_POSITION }
    private UpperSlidesState upperSlidesState = UpperSlidesState.INACTIVE;

    private void updateUpperSlides() {
        upperSlideRightPos  = upperSlideRight.getCurrentPosition();
        upperSlideLeftPos   = upperSlideLeft .getCurrentPosition();
        upperSlidesPos      = Math.max(upperSlideRightPos, upperSlideLeftPos);
        upperSlideRightVelo = upperSlideRight.getVelocity();
        upperSlideLeftVelo  = upperSlideLeft .getVelocity();

        switch (upperSlidesState) {
            case INACTIVE:
                break;
            case RUN_BY_VELOCITY:
                // prevent exceeding slide limits
                if (
                    (upperSlidesPos >= UPPER_SLIDES_TOP && upperSlidesTargetVelocity >= 0) ||
                    (upperSlidesPos <= UPPER_SLIDES_BOTTOM && upperSlidesTargetVelocity <= 0)
                ) {
                    return;
                }
                double rightPower = upperSlidesVelocityController.calculate(upperSlideRightVelo);
                double leftPower = upperSlidesVelocityController.calculate(upperSlideLeftVelo);

                upperSlideRight.setPower(rightPower);
                upperSlideLeft.setPower(leftPower);
                break;
            case RUN_TO_POSITION:
                break;
        }

        if (showTelemetry) {
            telemetry.addData("skibid toilet", 12);
        }
    }

    public int getUpperSlidesPos() { return upperSlidesPos; }
    public double getUpperSlidesVelo() { return upperSlideRight.getVelocity(); }

    /**
     * @param velocity [ticks/sec], positive is up
     */
    public void setUpperSlidesVelocity(double velocity) { // ticks/s
        int currentPos = getUpperSlidesPos();

        // prevent exceeding slide limits
        if ( (currentPos >= UPPER_SLIDES_TOP && velocity > 0) || (currentPos <= UPPER_SLIDES_BOTTOM && velocity < 0) )
            return;

        // set velocity
        for (DcMotorEx slide : upperSlides) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5,0,0,0));
            telemetry.addData("velo", velocity);
            slide.setVelocity(velocity);
        }
    }

    /**
     * @param pos [ticks]
     */
    public void setUpperSlidesPos(int pos) {
        // TODO: PIDF - find F, also use external PID
        int currentPos = getUpperSlidesPos();
        pos = clip(pos, UPPER_SLIDES_BOTTOM, UPPER_SLIDES_TOP);

        for (DcMotorEx slide : upperSlides) {
            slide.setTargetPosition(pos);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // TODO: test this
            slide.setPower(0.9);
//            slide.setVelocity( (pos > currentPos) ? VERT_SLIDES_DEFAULT_SPEED : 0.8*VERT_SLIDES_DEFAULT_SPEED);
        }
    }
    //endregion

    //region Upper Arm

    public void raiseArm()           { setUpperArmPos(UPPER_ARM_RAISED); }
    public void lowerArm()           { setUpperArmPos(UPPER_ARM_LOWERED); }

    public double getUpperArmPos() {
        return upperArmRight.getPosition();
    }

    public void setUpperArmPos(double pos) {
        upperArmRight.setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
        upperArmLeft .setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
    }
    //endregion

    //region Upper Claw

    public void closeUpperClaw()     { setUpperClawPos(UPPER_CLAW_CLOSED); }
    public void openUpperClaw()      { setUpperClawPos(UPPER_CLAW_OPEN); }

    public double getUpperClawPos()      { return upperClaw.getPosition(); }
    public void setUpperClawPos(double pos)      { upperClaw.setPosition(clip(pos, UPPER_CLAW_OPEN, UPPER_CLAW_CLOSED)); }
    //endregion

    //region Upper Claw Pitch

    public void raiseUpperClaw()     { setUpperClawPitchPos(UPPER_CLAW_UP); }
    public void lowerUpperClaw()     { setUpperClawPitchPos(UPPER_CLAW_DOWN); }

    public double getUpperClawPitchPos() { return upperClawPitch.getPosition(); }
    public void setUpperClawPitchPos(double pos) { upperClawPitch.setPosition(clip(pos, UPPER_CLAW_DOWN, UPPER_CLAW_UP)); }
    //endregion

    //region Lower Slides
    public void extendLowerSlides()  { setLowerSlidesPos(LOWER_SLIDES_EXTENDED); }
    public void retractLowerSlides() { setLowerSlidesPos(LOWER_SLIDES_RETRACTED); }

    public double getLowerSlidesPos() {
        return lowerSlideLeft.getPosition();
    }
    public void setLowerSlidesPos(double pos) {
        pos = clip(pos, LOWER_SLIDES_EXTENDED, LOWER_SLIDES_RETRACTED);
        for (Servo slide : lowerSlides)
            slide.setPosition(pos);
        // TODO: add camera for auto pickup
    }
    //endregion

    //region Lower Claw
    public void closeLowerClaw()     { setLowerClawPos(LOWER_CLAW_CLOSED); }
    public void openLowerClaw()      { setLowerClawPos(LOWER_CLAW_OPEN); }

    public double getLowerClawPos()      { return lowerClaw.getPosition(); }
    public void setLowerClawPos(double pos)      { lowerClaw.setPosition(clip(pos, LOWER_CLAW_CLOSED, LOWER_CLAW_OPEN)); }
    //endregion

    //region Lower Claw Pitch
    public void lowerLowerClaw()     { setLowerClawPitchPos(LOWER_CLAW_DOWN); }
    public void raiseLowerClaw()     { setLowerClawPitchPos(LOWER_CLAW_UP); }

    public double getLowerClawPitchPos() { return lowerClawPitch.getPosition(); }
    public void setLowerClawPitchPos(double pos) { lowerClawPitch.setPosition(clip(pos, LOWER_CLAW_DOWN, LOWER_CLAW_UP)); }
    //endregion

    //region Lower Claw Yaw
    public double getLowerClawYawPos()   { return lowerClawYaw.getPosition(); }
    public void setLowerClawYawPos(double pos)   { lowerClawYaw.setPosition(clip(pos, LOWER_CLAW_VERTICAL, LOWER_CLAW_HORIZONTAL_FLIPPED)); }
    //endregion

    // --------------------------------- CONTROL MACROS --------------------------------------

    //region Sample Transfer
    private enum TransferState { INACTIVE,  START, RETRACTING_SLIDES, TRANSFERRING }
    private TransferState transferState = TransferState.INACTIVE;
    private final ElapsedTime transferTimer = new ElapsedTime();
    private static double LOWER_SLIDES_RETRACT_DURATION = 0.5; // sec

    private void updateSampleTransferFSM() {
        switch (transferState) {
            case INACTIVE:
                break;
            case START:
                if ( specimenPickupState == SpecimenPickupState.INACTIVE ) {
                    setLowerClawPitchPos( LOWER_CLAW_PITCH_TRANSFER );
                    setLowerClawYawPos(   LOWER_CLAW_YAW_TRANSFER   );

                    transferTimer.reset();

                    if (getLowerSlidesPos() != LOWER_SLIDES_RETRACTED) {
                        retractLowerSlides();
                        transferState = TransferState.RETRACTING_SLIDES;
                    } else {
                        transferState = TransferState.TRANSFERRING;
                    }
                }
                break;
            case RETRACTING_SLIDES:
                if (transferTimer.time() > LOWER_SLIDES_RETRACT_DURATION) {
                    transferTimer.reset();
                    transferState = TransferState.TRANSFERRING;
                }
                break;
            case TRANSFERRING:
                // TODO:
                break;
        }
    }

    public void startSampleTransfer() {
        transferState = TransferState.START;
    }

    public void cancelSampleTransfer() {
        transferState = TransferState.INACTIVE;
    }

    //endregion

    //region Specimen Pickup
    private enum SpecimenPickupState { INACTIVE, START, RAISING_SLIDES, LOWERING_ARM }
    private SpecimenPickupState specimenPickupState = SpecimenPickupState.INACTIVE;
    private final ElapsedTime specimenPickupTimer = new ElapsedTime();
    private static double UPPER_ARM_SWING_DURATION = 0.5; // seconds

    private void updateSpecimenPickupFSM() {
        switch (specimenPickupState) {
            case INACTIVE:
                break;
            case START:
                if ( transferState == TransferState.INACTIVE ) {
                    setUpperSlidesPos(JamalTwo.UPPER_SLIDES_PICKUP_SPECIMEN);
                    specimenPickupState = SpecimenPickupState.RAISING_SLIDES;
                }
                break;
            case RAISING_SLIDES:
                if ( getUpperSlidesPos() > UPPER_SLIDES_MIN_ARM_CLEARANCE) {
                    setUpperArmPos(UPPER_ARM_PICKUP_SPECIMEN);
                    setUpperClawPos(UPPER_CLAW_PITCH_PICKUP_SPECIMEN);
                    specimenPickupTimer.reset();
                    specimenPickupState = SpecimenPickupState.LOWERING_ARM;
                }
                break;
            case LOWERING_ARM:
                if ( specimenPickupTimer.time() > UPPER_ARM_SWING_DURATION)
                    specimenPickupState = SpecimenPickupState.INACTIVE;
                break;
        }
    }

    public void startSpecimenPickup() {
        specimenPickupState = SpecimenPickupState.START;
    }

    public void cancelSpecimenPickup() {
        specimenPickupState = SpecimenPickupState.INACTIVE;
    }

    //endregion

    public void prepareSpecimenDeposit() {
        setUpperSlidesPos(    UPPER_SLIDES_ABOVE_HIGH_CHAMBER );
        setUpperArmPos(       UPPER_ARM_DEPOSIT_SPECIMEN      );
        setUpperClawPitchPos(UPPER_CLAW_PITCH_DEPOSIT_SPECIMEN);
    }

    public void prepareSamplePickup() {
        extendLowerSlides();
        setLowerClawPitchPos( LOWER_CLAW_DOWN       );
        setLowerClawYawPos(   LOWER_CLAW_HORIZONTAL );
    }

    public void prepareSampleDeposit() {
        setUpperSlidesPos(    UPPER_SLIDES_DEPOSIT_SAMPLE );
        setUpperArmPos(       UPPER_ARM_DEPOSIT_SAMPLE    );
        setUpperClawPitchPos(UPPER_CLAW_PITCH_DEPOSIT_SAMPLE);
    }

}
