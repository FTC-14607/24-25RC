package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@Config
@TeleOp(name = "JamalTwo Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    public static double UPPER_SLIDE_MAX_SPEED = 800; // ticks / sec
    public static double LOWER_SLIDE_MAX_SPEED = 0.2; // servo position / sec

    JamalTwo robot;

    boolean showTelemetry = true;

    boolean holdingUpperSlides = true;
    int holdUpperSlidesPos = 0;
    boolean upperClawOpen = true;
    boolean lowerClawOpen = true;

    ElapsedTime lowerSlidesVeloTimer = new ElapsedTime();

    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new JamalTwo(this);
        robot.maxDrivePower = 0.9;

        double horizontalSlidePos = JamalTwo.LOWER_SLIDES_RETRACTED;
        int verticalSlidePos = JamalTwo.UPPER_SLIDES_BOTTOM;

        initPositions();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();
            robot.update();

            // TODO: instead of passing a gamepad, make variables like upperClawInput and read

            // semi-automated control
            if      (gamepad2.a) {
                robot.startSampleTransfer();
            }
            else if (gamepad2.b) {
                robot.startSpecimenPickup();
            }
            else if (gamepad2.x) {
                robot.prepareSampleDeposit();
            }
            else if (gamepad2.y) {
                robot.prepareSamplePickup();
            }
            else if (gamepad2.right_bumper) {
                robot.prepareSpecimenDeposit();
            }
            else if (gamepad2.left_bumper) {
            }

            // manual control of each individual part
            controlUpperSlides(    gamepad2 );
            controlUpperArm(       gamepad2 );
            controlUpperClaw(      gamepad2 );
            controlUpperClawPitch( gamepad2 );

//            controlLowerSlides(    gamepad1 );
            controlLowerClaw(      gamepad1 );
            controlLowerClawPitch( gamepad1 );
            controlLowerClawYaw(   gamepad1 );
            controlDriveTrain(     gamepad1 );


            if (showTelemetry) {
                telemetry.addData("Vertical Slide Position", robot.getUpperSlidesPos());
                telemetry.addData("vert hold pos", holdUpperSlidesPos);
                telemetry.addData("Upper Arm Position", robot.getUpperArmPos());
                telemetry.addData("Horizontal Slide Position", robot.getLowerSlidesPos());

                telemetry.addData("pos pid", robot.upperSlideRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
                telemetry.addData("velo pid", robot.upperSlideRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

                telemetry.addData("Max Drive Power", robot.maxDrivePower);
                telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
                telemetry.update();
            }
        }

    }

    public void initPositions() {

    }

    public void controlUpperSlides(Gamepad gamepad) {
        double input = -gamepad.left_stick_y;

        if (input != 0) {
            double slideVelo = input * UPPER_SLIDE_MAX_SPEED;
            telemetry.addData("slidevelo", slideVelo);
//            for (DcMotorEx slide : robot.upperSlides)
//                slide.setPower(slideVelo);
            robot.setUpperSlidesVelocity(slideVelo);

            holdingUpperSlides = false;
        }
        else {
            if ( !holdingUpperSlides ) {
                holdUpperSlidesPos = robot.getUpperSlidesPos();
                holdingUpperSlides = true;
            }
//            for (DcMotorEx slide : robot.upperSlides)
//                slide.setPower(0);
            robot.setUpperSlidesPos(holdUpperSlidesPos);
        }
    }

    public void controlLowerSlides(Gamepad gamepad) {
        double input = gamepad.right_stick_x;

        double deltaTime = lowerSlidesVeloTimer.time(); // time since last iteration
        double nextSlidePos = robot.getLowerSlidesPos() + input * deltaTime * LOWER_SLIDE_MAX_SPEED;
        robot.setLowerSlidesPos(nextSlidePos);

        lowerSlidesVeloTimer.reset();
    }

    public void controlDriveTrain(Gamepad gamepad) {

        // TODO: field centric

        // change max drive power
        if      (gamepad.a) robot.maxDrivePower = 0.3;
        else if (gamepad.b) robot.maxDrivePower = 0.5;
        else if (gamepad.y) robot.maxDrivePower = 0.7;
        else if (gamepad.x) robot.maxDrivePower = 0.9;

        // set drivetrain power (flipped cuz i thoguht thw wrong side was forward 3050 2075
        double throttle = -gamepad.left_stick_y * -1;
        double strafe   = -gamepad.left_stick_x;
        double rotate   = gamepad.right_stick_x;

        // dpad for easy orthogonal movement
        if      (gamepad.dpad_up)    throttle =  1;
        else if (gamepad.dpad_down)  throttle = -1;
        else if (gamepad.dpad_right) strafe   =  1;
        else if (gamepad.dpad_left)  strafe   = -1;

        // TODO: setup rotation corrections

        robot.drive(throttle, strafe, rotate);
    }

    public void controlUpperArm(Gamepad gamepad) {

    }

    public void controlUpperClaw(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            if (upperClawOpen) { robot.closeUpperClaw(); upperClawOpen = false; }
            else               { robot.openUpperClaw();  upperClawOpen = true;  }
        }
    }

    public void controlUpperClawPitch(Gamepad gamepad) {

    }


    public void controlLowerClaw(Gamepad gamepad) {
        if (gamepad.dpad_down) {
            if (lowerClawOpen) { robot.closeLowerClaw(); lowerClawOpen = false; }
            else               { robot.openLowerClaw();  lowerClawOpen = true;  }
        }
    }

    public void controlLowerClawPitch(Gamepad gamepad) {

    }

    public void controlLowerClawYaw(Gamepad gamepad) {

    }

}
