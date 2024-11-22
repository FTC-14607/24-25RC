package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.odometry.FTCLibOdometry;

@Config
public class CardboardOne extends MecanumDrive {

    // actuators and sensors
    public DcMotorEx vertSlideRight;
    public DcMotorEx vertSlideLeft;
    public DcMotorEx[] vertSlides;
    public Servo upperArm;
    public Servo dumper;

    public Servo horiSlideRight; // axon servos
    public Servo horiSlideLeft;
    public Servo[] horiSlides;
    public Servo clawSample;
    public Servo clawSampleYaw;
    public Servo clawSamplePitch;

    public Servo clawSpecimen;

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

    //  constants
    public final static RobotDimensions DIMENSIONS = new RobotDimensions(
            18, 18, 18, 9.6, 537.68984);
    public static double TRACK_WIDTH = 8.2087;        // 20.85 cm TODO
    public static double CENTER_WHEEL_OFFSET = 0;   // 0 cm TODO

    // GoBilda Odometry Pod; 2000 ticks/rev, 48 mm diameter; somehow pods are reversed so * -1
    public static double ODO_TICKS_TO_INCHES = 3.14159 * (4.8 / 2.54) / 2000 * -1;

    public static int VERT_SLIDES_BOTTOM = 0; // ticks
    public static int VERT_SLIDES_TOP = 100;
    public static double VERT_SLIDES_DEFAULT_SPEED = 100;
    public static double VERT_SLIDES_INCHES_TO_TICKS = -1;
    public static double UPPER_ARM_LOWERED = 0.47; // servo position [0, 1]
    public static double UPPER_ARM_RAISED = 1;
    public static double DUMPER_DOWN = 0;
    public static double DUMPER_TOP = 1;
    public static double DUMPER_DUMP = 0.5;

    public static double HORI_SLIDES_RETRACTED = 0.86;
    public static double HORI_SLIDES_EXTENDED = 0.6;
    public static double SAMPLE_CLAW_CLOSED = 0.84;
    public static double SAMPLE_CLAW_OPEN = 0.64;
    public static double SAMPLE_CLAW_HORIZONTAL = 0.46; // to pick up vertical samples
    public static double SAMPLE_CLAW_VERTICAL = 0.14;
    public static double SAMPLE_CLAW_VERTICAL_FLIPPED = 0.78;
    public static double SAMPLE_CLAW_DOWN = 0;
    public static double SAMPLE_CLAW_UP = 0.55;

    public static double SPECIMEN_CLAW_CLOSED = 0.1;
    public static double SPECIMEN_CLAW_OPEN = 0;


    public CardboardOne(LinearOpMode opmode) {
        super(opmode);
        dimensions = DIMENSIONS;

        // connect to hardware
        vertSlideRight  = hardwareMap.get(DcMotorEx.class, "vertSlideRight");
        vertSlideLeft   = hardwareMap.get(DcMotorEx.class, "vertSlideLeft");
        upperArm        = hardwareMap.get(Servo.class,     "upperArm");
        dumper          = hardwareMap.get(Servo.class,     "dumper");

        horiSlideRight  = hardwareMap.get(Servo.class,     "horiSlideRight");
        horiSlideLeft   = hardwareMap.get(Servo.class,     "horiSlideLeft");
        clawSample      = hardwareMap.get(Servo.class,     "clawSample");
        clawSampleYaw   = hardwareMap.get(Servo.class,     "clawSampleYaw");
        clawSamplePitch = hardwareMap.get(Servo.class,     "clawSamplePitch");

        clawSpecimen    = hardwareMap.get(Servo.class,     "clawSpecimen");

        // TODO: attach odo
//        odoRight = new MotorEx(hardwareMap, "frontLeft"); // encoders are plugged in next to motors
//        odoLeft  = new MotorEx(hardwareMap, "backRight");
//        odoPerp  = new MotorEx(hardwareMap, "frontRight");

        // setup hardware
        dumper.setDirection(Servo.Direction.REVERSE); // dumper servo is on opposite side of arm servo

        vertSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        vertSlides = new DcMotorEx[] { vertSlideRight, vertSlideLeft };
        horiSlideRight.setDirection(Servo.Direction.REVERSE);
        horiSlides = new Servo[] { horiSlideRight, horiSlideLeft };

        setZeroPowerBehavior(vertSlides, DcMotor.ZeroPowerBehavior.BRAKE); // TODO: experiment with this
//        setRunMode(vertSlides, DcMotor.RunMode.RUN_TO_POSITION);
        setDirection(vertSlides, DcMotorSimple.Direction.FORWARD);

//        odo = new FTCLibOdometry(odoLeft, odoRight, odoPerp, TRACK_WIDTH, CENTER_WHEEL_OFFSET, ODO_TICKS_TO_INCHES);
//        odo.init();

        imu.resetYaw();

        // initialize controllers
        throttleController = new PIDFController(tP, tI, tD, tF);
        strafeController = new PIDFController(sP, sI, sD, sF);
        headingController = new PIDFController(rP, rI, rD, rF);

        throttleController.setTolerance(tTol); // inches
        strafeController  .setTolerance(sTol); // inches
        headingController.setTolerance(rTol);   // degrees

        translationController = new PIDFController(trP, trI, trD, trF);
        translationController.setTolerance(trTol);
    }

    // ---------------------------------------- SENSORS --------------------------------------------

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

    // -------------------------------------- INTERACTORS ------------------------------------------

    public double getUpperArmPos()         { return upperArm       .getPosition(); }
    public double getDumperPos()           { return dumper         .getPosition(); }
    public double getSampleClawPos()       { return clawSample     .getPosition(); }
    public double getSampleClawYawPos()    { return clawSampleYaw  .getPosition(); }
    public double getSampleClawPitchPos()  { return clawSamplePitch.getPosition(); }
    public double getSpecimenClawPos()     { return clawSpecimen   .getPosition(); }

    // TODO: maybe use Range.scale
    public void setDumperPos          (double pos) { dumper         .setPosition(Range.clip(pos, DUMPER_DOWN,        DUMPER_TOP        )); }
    public void setSampleClawPos      (double pos) { clawSample     .setPosition(Range.clip(pos, SAMPLE_CLAW_CLOSED,   SAMPLE_CLAW_OPEN  )); }
    public void setSampleClawYawPos   (double pos) { clawSampleYaw  .setPosition(Range.clip(pos, SAMPLE_CLAW_VERTICAL,SAMPLE_CLAW_VERTICAL_FLIPPED)); }
    public void setSampleClawPitchPos (double pos) { clawSamplePitch.setPosition(Range.clip(pos, SAMPLE_CLAW_DOWN,     SAMPLE_CLAW_UP    )); }
    public void setSpecimenClawPos    (double pos) { clawSpecimen   .setPosition(Range.clip(pos, SPECIMEN_CLAW_OPEN, SPECIMEN_CLAW_CLOSED)); }

    public void raiseArm()                { setUpperArmPos(UPPER_ARM_RAISED); }
    public void lowerArm()                { setUpperArmPos(UPPER_ARM_LOWERED); }
    public void dumpDumper()              { setDumperPos(           DUMPER_DUMP           ); }
    public void resetDumper()             { setDumperPos(DUMPER_DOWN); }
    public void extendHorizontalSlides()  { setHorizontalSlidesPos( HORI_SLIDES_EXTENDED  ); }
    public void retractHorizontalSlides() { setHorizontalSlidesPos( HORI_SLIDES_RETRACTED ); }
    public void closeSampleClaw()         { setSampleClawPos(       SAMPLE_CLAW_CLOSED    ); }
    public void openSampleClaw()          { setSampleClawPos(       SAMPLE_CLAW_OPEN      ); }
    public void lowerSampleClaw()         { setSampleClawPitchPos(  SAMPLE_CLAW_DOWN      ); }
    public void raiseSampleClaw()         { setSampleClawPitchPos(  SAMPLE_CLAW_UP        ); }
    public void closeSpecimenClaw()       { setSpecimenClawPos(     SPECIMEN_CLAW_CLOSED  ); }
    public void openSpecimenClaw()        {  setSpecimenClawPos(    SPECIMEN_CLAW_OPEN    ); }

    public double getHorizontalSlidesPos() { return horiSlideLeft .getPosition(); }

    public void setHorizontalSlidesPos(double pos) {
        pos = Range.clip(pos, HORI_SLIDES_EXTENDED, HORI_SLIDES_RETRACTED);
        for (Servo slide : horiSlides)
            slide.setPosition(pos);
        // TODO: add camera for auto pickup
    }

    public int getVerticalSlidePos() {
        // return vertSlideRight.getCurrentPosition();
        // Math.max since one motor may be broken, returning 0
        return Math.max(vertSlideLeft.getCurrentPosition(), vertSlideRight.getCurrentPosition());
        // TODO: test loop speed
    }

    /**
     * @param velocity [ticks/sec], positive is up
     */
    public void setVerticalSlidesVelocity(double velocity) { // ticks/s
        int currentPos = getVerticalSlidePos();

        // prevent exceeding slide limits
        if ( (currentPos >= VERT_SLIDES_TOP && velocity > 0) || (currentPos <= VERT_SLIDES_BOTTOM && velocity < 0) )
            return;

        // set velocity
        for (DcMotorEx slide : vertSlides) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setVelocity(velocity);
        }
    }


    /**
     * @param pos [ticks]
     */
    public void setVerticalSlidesPos(int pos) {
        int currentPos = getVerticalSlidePos();
        pos = Range.clip(pos, VERT_SLIDES_BOTTOM, VERT_SLIDES_TOP);

        for (DcMotorEx slide : vertSlides) {
            slide.setTargetPosition(pos);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // TODO: test this
            slide.setPower(0.2);
//            slide.setVelocity( (pos > currentPos) ? VERT_SLIDES_DEFAULT_SPEED : 0.8*VERT_SLIDES_DEFAULT_SPEED);
        }
    }

    /**
     * @param setPos [0-1]
     */
    public void setUpperArmPos(double setPos) { // needs to keep dumper level while raising arm
        setPos = Range.clip(setPos, UPPER_ARM_LOWERED, UPPER_ARM_RAISED);

        double currentPos = getUpperArmPos();
        double dumperPos = getDumperPos();
        double posChange = setPos - currentPos;

        upperArm.setPosition(setPos);
        setDumperPos(dumperPos - posChange);  // dumper rotates the opposite way by the same amount
    }


    // ------------------------------------------- DRIVE -------------------------------------------

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
     * @param displacement (inches, inches degrees)
     */
    public void moveBy(Pose2d displacement, double holdTime) {
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
    public void move(double straightDistance, double strafeDistance, double rotateAngle, double holdTime) {
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
