package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.hardware.LinearSlideMotor;
import org.firstinspires.ftc.teamcode.util.odometry.AutoDriver;
import org.firstinspires.ftc.teamcode.util.hardware.Encoder;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.odometry.MecanumThreeWheelOdometryDriver;
import org.firstinspires.ftc.teamcode.util.odometry.RobotLocalizer;

@Config
public class JamalTwo extends MecanumDrive {

    //region hardware
    // * drivetrain: goBILDA 96mm Mecanum Wheels and goBILDA 5203 312 RPM Motors
    public LinearSlideMotor upperSlideRight; // goBILDA 5203 312 RPM
    public LinearSlideMotor upperSlideLeft;  // same
    public Servo upperArmRight;   // Axon Mini
    public Servo upperArmLeft;    // same
    public Servo upperClaw;       // goBILDA Torque
    public Servo upperClawPitch;  // same

    public Servo lowerSlideRight; // same
    public Servo lowerSlideLeft;  // same
    public Servo lowerClaw;       // same
    public Servo lowerClawYaw;    // same
    public Servo lowerClawPitch;  // same

    public Encoder odoRight; // goBILDA Swingarm Odometry Pod (2000 ticks/rot, 48mm wheel)
    public Encoder odoLeft;  // same
    public Encoder odoPerp;  // same
    //endregion
    //region control
    public LinearSlideMotor[] upperSlides;
    public int upperSlidesPos;
    public double upperSlidesMaxPower = 0.9;
    public static PIDFCoefficients upperSlidesVelocityPIDFCoefficients =
            new PIDFCoefficients(0.00032,0.0,0.0,0.00005);
    public static double upperSlidesPositionGain = 3;

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
    public final static int ODO_TICKS_PER_ROTATION = 2000;
    public final static double ODO_WHEEL_DIAMETER = centimetersToInches(48. / 10);

    public final static int
        UPPER_SLIDES_BOTTOM = 0, // ticks
        UPPER_SLIDES_TOP = 4550,
        UPPER_SLIDES_TRANSFER = 0,
        UPPER_SLIDES_MIN_ARM_CLEARANCE = 1700, // minimum height at which the upper arm can go fully down
        UPPER_SLIDES_PICKUP_SPECIMEN = 1860,
        UPPER_SLIDES_DEPOSIT_SPECIMEN = UPPER_SLIDES_TOP,
        UPPER_SLIDES_DEPOSIT_SAMPLE = 2950;
    public static double UPPER_SLIDES_DEFAULT_SPEED = 100;
    public static double UPPER_SLIDES_INCHES_TO_TICKS = -1;

    public final static double
        UPPER_ARM_LOWERED = 0.02, // servo position [0, 1]
        UPPER_ARM_RAISED = 1,
        UPPER_ARM_REST = 0.466,
        UPPER_ARM_TRANSFER = 0.55,
        UPPER_ARM_PICKUP_SPECIMEN = 0.04,
        UPPER_ARM_DEPOSIT_SPECIMEN = 0.04,
        UPPER_ARM_DEPOSIT_SAMPLE = 1;

    public final static double
        UPPER_CLAW_CLOSED = 0.862,
        UPPER_CLAW_OPEN = 0.735;

    public final static double
        UPPER_CLAW_DOWN = 0,
        UPPER_CLAW_UP = 1,
        UPPER_CLAW_PITCH_TRANSFER = 0.3228,
        UPPER_CLAW_PITCH_PICKUP_SPECIMEN = 0.282,
        UPPER_CLAW_PITCH_DEPOSIT_SPECIMEN = 0.282,
        UPPER_CLAW_PITCH_DEPOSIT_SAMPLE = 0.5428;

    public final static double
        LOWER_SLIDES_RETRACTED = 0.1522,
        LOWER_SLIDES_EXTENDED = 0.422,
        LOWER_SLIDES_RETRACT_DURATION = 0.7, // sec
        LOWER_SLIDES_EXTEND_DURATION = 0.3;

    public final static double
        LOWER_CLAW_CLOSED = 0.83,
        LOWER_CLAW_OPEN = 0.55;

    public final static double
        LOWER_CLAW_YAW_HORIZONTAL = 0.0574, // to pick up vertical samples
        LOWER_CLAW_YAW_45DEGREES = 0.218,
        LOWER_CLAW_YAW_VERTICAL = 0.3786,
        LOWER_CLAW_YAW_135DEGREES = 0.5433,
        LOWER_CLAW_YAW_HORIZONTAL_FLIPPED = 0.708,
        LOWER_CLAW_YAW_TRANSFER = LOWER_CLAW_YAW_HORIZONTAL_FLIPPED;

    public final static double
        LOWER_CLAW_DOWN = 0.415,
        LOWER_CLAW_UP = 1,
        LOWER_CLAW_PITCH_TRANSFER = LOWER_CLAW_UP;
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
        upperSlideRight = new LinearSlideMotor(hardwareMap.get(DcMotorEx.class, "upperSlideRight"));
        upperSlideLeft  = new LinearSlideMotor(hardwareMap.get(DcMotorEx.class, "upperSlideLeft"));
        upperArmRight   = hardwareMap.get(Servo.class, "upperArmRight");
        upperArmLeft    = hardwareMap.get(Servo.class, "upperArmLeft");
        upperClaw       = hardwareMap.get(Servo.class, "upperClaw");
        upperClawPitch  = hardwareMap.get(Servo.class, "upperClawPitch");

        lowerSlideRight = hardwareMap.get(Servo.class, "lowerSlideRight");
        lowerSlideLeft  = hardwareMap.get(Servo.class, "lowerSlideLeft");
        lowerClaw       = hardwareMap.get(Servo.class, "lowerClaw");
        lowerClawYaw    = hardwareMap.get(Servo.class, "lowerClawYaw");
        lowerClawPitch  = hardwareMap.get(Servo.class, "lowerClawPitch");

        odoRight = new Encoder(frontLeft); // encoders are plugged in next to motors
        odoLeft =  new Encoder(backRight);
        odoPerp =  new Encoder(frontRight);

        // initialization and setup
        upperSlides = new LinearSlideMotor[] {upperSlideRight, upperSlideLeft};
        for (LinearSlideMotor slide : upperSlides) {
            slide.addControl(UPPER_SLIDES_BOTTOM, UPPER_SLIDES_TOP, voltageScaler, upperSlidesMaxPower,
                    upperSlidesPositionGain,
                    upperSlidesVelocityPIDFCoefficients.p,
                    upperSlidesVelocityPIDFCoefficients.i,
                    upperSlidesVelocityPIDFCoefficients.d,
                    upperSlidesVelocityPIDFCoefficients.f
            );
        }
        upperSlideRight.setDirection(DcMotor.Direction.REVERSE);

        upperArmRight.setDirection(Servo.Direction.REVERSE);

        // TODO: internal servo scaling
//        upperClaw.scaleRange(UPPER_CLAW_CLOSED, UPPER_CLAW_OPEN); //
        upperClawPitch.setDirection(Servo.Direction.REVERSE);

        lowerSlides = new Servo[] {lowerSlideRight, lowerSlideLeft};
        lowerSlideRight.setDirection(Servo.Direction.REVERSE);

        odoLeft.setDirection(Encoder.Direction.REVERSE);
//        odoPerp.setDirection(Encoder.Direction.REVERSE);

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
     * Re-reads all sensors and updates all finite-state machines
     */
    public void update() {
        updatePose();
        updateUpperSlides();
        updateTransferFSM();
        updatePrepareSpecimenPickupFSM();
        updatePrepareSamplePickupFSM();
        updatePrepareSampleDepositFSM();
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
    private void updateUpperSlides() {
        upperSlideRight.update();
        upperSlideLeft.update();

        upperSlidesPos = Math.max(upperSlideRight.getLastPosition(), upperSlideLeft.getLastPosition());

        if (showTelemetry) {
//            telemetry.addData("outpower", upperSlideRight.totpower);
//            telemetry.addData("rmode", upperSlideRight.rmode);
//            telemetry.addData("Slide FF Power", "%5.2f | %5.2f", upperSlideRight.ffpower, upperSlideLeft.ffpower);
//            telemetry.addData("Slide FB Power", "%5.2f | %5.2f", upperSlideRight.fbpower, upperSlideLeft.fbpower);
            telemetry.addData("Slide Power", "%5.2f | %5.2f", upperSlideRight.getPower(), upperSlideLeft.getPower());
            telemetry.addData("Slide Positions", "%d | %d", upperSlideRight.getLastPosition(), upperSlideLeft.getLastPosition());
            telemetry.addData("Slide Targets", "%d | %d", upperSlideRight.getTargetPositionEx(), upperSlideLeft.getTargetPositionEx());

            telemetry.addData("Slide Velocity", "%5.2f | %5.2f", upperSlideRight.getLastVelocity(), upperSlideLeft.getLastVelocity());
            telemetry.addData("Slide Velocity Targets", "%5.2f | %5.2f", upperSlideRight.getTargetVelocityEx(), upperSlideLeft.getTargetVelocityEx());
        }
    }

    public int getUpperSlidesPos() { return upperSlidesPos; }
    public double getUpperSlidesVelo() { return upperSlideRight.getLastVelocity(); }

    /**
     * @param pos [ticks]
     */
    public void setUpperSlidesPos(int pos) {
        upperSlideRight.setTargetPositionEx(pos);
        upperSlideLeft.setTargetPositionEx(pos);
    }
    public void setUpperSlidesVelocity(double velocity) { // ticks/s
        upperSlideRight.setTargetVelocityEx(velocity);
        upperSlideLeft.setTargetVelocityEx(velocity);
    }
    //endregion

    //region Upper Arm
    public void raiseArm() { setUpperArmPos(UPPER_ARM_RAISED); }
    public void lowerArm() { setUpperArmPos(UPPER_ARM_LOWERED); }

    public double getUpperArmPos() {
        return upperArmRight.getPosition();
    }

    public void setUpperArmPos(double pos) {
        upperArmRight.setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
        upperArmLeft .setPosition(clip(pos, UPPER_ARM_RAISED, UPPER_ARM_LOWERED));
    }
    //endregion

    //region Upper Claw
    public void closeUpperClaw() { setUpperClawPos(UPPER_CLAW_CLOSED); }
    public void openUpperClaw()  { setUpperClawPos(UPPER_CLAW_OPEN); }

    public double getUpperClawPos() { return upperClaw.getPosition(); }
    public void setUpperClawPos(double pos) { upperClaw.setPosition(clip(pos, UPPER_CLAW_OPEN, UPPER_CLAW_CLOSED)); }
    //endregion

    //region Upper Claw Pitch
    public void raiseUpperClaw() { setUpperClawPitchPos(UPPER_CLAW_UP); }
    public void lowerUpperClaw() { setUpperClawPitchPos(UPPER_CLAW_DOWN); }

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
    public void closeLowerClaw() { setLowerClawPos(LOWER_CLAW_CLOSED); }
    public void openLowerClaw()  { setLowerClawPos(LOWER_CLAW_OPEN); }

    public double getLowerClawPos() { return lowerClaw.getPosition(); }
    public void setLowerClawPos(double pos) { lowerClaw.setPosition(clip(pos, LOWER_CLAW_CLOSED, LOWER_CLAW_OPEN)); }
    //endregion

    //region Lower Claw Pitch
    public void lowerLowerClaw() { setLowerClawPitchPos(LOWER_CLAW_DOWN); }
    public void raiseLowerClaw() { setLowerClawPitchPos(LOWER_CLAW_UP); }

    public double getLowerClawPitchPos() { return lowerClawPitch.getPosition(); }
    public void setLowerClawPitchPos(double pos) { lowerClawPitch.setPosition(clip(pos, LOWER_CLAW_DOWN, LOWER_CLAW_UP)); }
    //endregion

    //region Lower Claw Yaw
    public double getLowerClawYawPos() { return lowerClawYaw.getPosition(); }
    public void setLowerClawYawPos(double pos) { lowerClawYaw.setPosition(clip(pos, 0, 1)); }
    //endregion

    // --------------------------------- CONTROL MACROS --------------------------------------

    public boolean allMacrosInactive() {
        return
                transferState == TransferState.INACTIVE &&
                prepareSpecimenPickupState == PrepareSpecimenPickupState.INACTIVE &&
                prepareSampleDepositState == PrepareSampleDepositState.INACTIVE &&
                prepareSamplePickupState == PrepareSamplePickupState.INACTIVE;
    }

    public void cancelAllMacros() {
        cancelTransfer();
        cancelPrepareSpecimenPickup();
        cancelPrepareSamplePickup();
        cancelPrepareSampleDeposit();
    }

    //region Transfer sample between claws
    private enum TransferState { INACTIVE,  START, RAISING_ARM, RETRACTING_SLIDES, TRANSFERRING }
    private TransferState transferState = TransferState.INACTIVE;
    private final ElapsedTime transferTimer = new ElapsedTime();
    public static double RAISE_UPPER_ARM_TRANSFER_DURATION = 0.3;

    private void updateTransferFSM() {
        switch (transferState) {
            case INACTIVE:
                break;
            case START:
                openUpperClaw();
                setUpperSlidesPos( UPPER_SLIDES_TRANSFER );
                setUpperArmPos( UPPER_ARM_TRANSFER );

                transferTimer.reset();
                transferState = TransferState.RAISING_ARM;
            case RAISING_ARM:
                if (transferTimer.time() < RAISE_UPPER_ARM_TRANSFER_DURATION)
                    break;

                setUpperClawPitchPos( UPPER_CLAW_PITCH_TRANSFER );
                setLowerClawPitchPos( LOWER_CLAW_PITCH_TRANSFER );
                setLowerClawYawPos(   LOWER_CLAW_YAW_TRANSFER   );

                transferTimer.reset();

                if (getLowerSlidesPos() != LOWER_SLIDES_RETRACTED) {
                    retractLowerSlides();
                    transferState = TransferState.RETRACTING_SLIDES;
                } else {
                    transferState = TransferState.TRANSFERRING;
                }
                break;
            case RETRACTING_SLIDES:
                if (transferTimer.time() < LOWER_SLIDES_RETRACT_DURATION)
                    break;

                transferTimer.reset();
                transferState = TransferState.TRANSFERRING;
                break;
            case TRANSFERRING:
                closeUpperClaw();
                openLowerClaw();
                transferState = TransferState.INACTIVE;
                break;
        }
    }

    public void startTransfer() {
        if (allMacrosInactive())
            transferState = TransferState.START;
    }

    public void cancelTransfer() {
        transferState = TransferState.INACTIVE;
    }
    //endregion

    //region Prepare Specimen Pickup
    private enum PrepareSpecimenPickupState { INACTIVE, START, RAISING_SLIDES }
    private PrepareSpecimenPickupState prepareSpecimenPickupState = PrepareSpecimenPickupState.INACTIVE;

    private void updatePrepareSpecimenPickupFSM() {
        switch (prepareSpecimenPickupState) {
            case INACTIVE:
                break;
            case START:
                setUpperSlidesPos(JamalTwo.UPPER_SLIDES_PICKUP_SPECIMEN);
                prepareSpecimenPickupState = PrepareSpecimenPickupState.RAISING_SLIDES;
                break;
            case RAISING_SLIDES:
                if ( getUpperSlidesPos() > UPPER_SLIDES_MIN_ARM_CLEARANCE) {
                    openUpperClaw();
                    setUpperArmPos(UPPER_ARM_PICKUP_SPECIMEN);
                    setUpperClawPitchPos(UPPER_CLAW_PITCH_PICKUP_SPECIMEN);
                    prepareSpecimenPickupState = PrepareSpecimenPickupState.INACTIVE;
                }
                break;
        }
    }

    public void startPrepareSpecimenPickup() {
        if (allMacrosInactive())
            prepareSpecimenPickupState = PrepareSpecimenPickupState.START;
    }

    public void cancelPrepareSpecimenPickup() {
        prepareSpecimenPickupState = PrepareSpecimenPickupState.INACTIVE;
    }
    //endregion

    //region Prepare Sample Pickup
    private enum PrepareSamplePickupState { INACTIVE,  START, EXTENDING_SLIDES }
    private PrepareSamplePickupState prepareSamplePickupState = PrepareSamplePickupState.INACTIVE;
    private final ElapsedTime prepareSamplePickupTimer = new ElapsedTime();

    private void updatePrepareSamplePickupFSM() {
        switch (prepareSamplePickupState) {
            case INACTIVE:
                break;
            case START:
                extendLowerSlides();
                prepareSamplePickupState = PrepareSamplePickupState.EXTENDING_SLIDES;
                prepareSamplePickupTimer.reset();
                break;
            case EXTENDING_SLIDES:
                if (prepareSamplePickupTimer.time() > LOWER_SLIDES_EXTEND_DURATION) {
                    openLowerClaw();
                    setLowerClawPitchPos( LOWER_CLAW_DOWN );
                    setLowerClawYawPos(LOWER_CLAW_YAW_HORIZONTAL);
                    prepareSamplePickupState = PrepareSamplePickupState.INACTIVE;
                }
                break;
        }
    }

    public void startPrepareSamplePickup() {
        if (allMacrosInactive())
            prepareSamplePickupState = PrepareSamplePickupState.START;
    }

    public void cancelPrepareSamplePickup() {
        prepareSamplePickupState = PrepareSamplePickupState.INACTIVE;
    }
    //endregion

    //region Prepare Sample Deposit
    private enum PrepareSampleDepositState { INACTIVE, START, RAISING_SLIDES }
    private PrepareSampleDepositState prepareSampleDepositState = PrepareSampleDepositState.INACTIVE;

    public void updatePrepareSampleDepositFSM() {
        switch (prepareSampleDepositState) {
            case INACTIVE:
                break;
            case START:
                setUpperSlidesPos( UPPER_SLIDES_DEPOSIT_SAMPLE );
                setUpperArmPos(       UPPER_ARM_DEPOSIT_SAMPLE    );
                setUpperClawPitchPos(UPPER_CLAW_PITCH_DEPOSIT_SAMPLE);
                prepareSampleDepositState = PrepareSampleDepositState.RAISING_SLIDES;
                break;
            case RAISING_SLIDES:
                if ( getUpperSlidesPos() > UPPER_SLIDES_DEPOSIT_SAMPLE ) {
                    prepareSampleDepositState = PrepareSampleDepositState.INACTIVE;
                }
        }
    }

    public void startPrepareSampleDeposit() {
        if (allMacrosInactive())
            prepareSampleDepositState = PrepareSampleDepositState.START;
    }

    public void cancelPrepareSampleDeposit() {
        prepareSampleDepositState = PrepareSampleDepositState.INACTIVE;
    }
    //endregion

    public void prepareSpecimenDeposit() {
        if (!allMacrosInactive())
            return;
        setUpperSlidesPos(UPPER_SLIDES_DEPOSIT_SPECIMEN);
        setUpperArmPos( UPPER_ARM_DEPOSIT_SPECIMEN );
        setUpperClawPitchPos( UPPER_CLAW_PITCH_DEPOSIT_SPECIMEN );

    }

}