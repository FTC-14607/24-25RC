package org.firstinspires.ftc.teamcode.teleops;

import android.icu.text.CaseMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalOne;
import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    JamalTwo robot;
    public static double UPPER_SLIDE_MAX_SPEED = 100; // ticks / sec
    public static double LOWER_SLIDE_MAX_SPEED = 0.2; // servo position / sec

    ElapsedTime loopTimer = new ElapsedTime();

    public void runOpMode() {
        robot = new JamalTwo(this);
        robot.maxDrivePower = 0.9;

        double horizontalSlidePos = JamalTwo.LOWER_SLIDES_RETRACTED;
        int verticalSlidePos = JamalTwo.UPPER_SLIDES_BOTTOM;

        initPositions();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            // TODO: univeral robot.update() method
            // TODO: add finite state machine so things don't break


            moveUpperSlides(gamepad2);
            moveLowerSlides(gamepad1);
            moveDriveTrain(gamepad1);


            if (true) {
                telemetry.addData("Vertical Slide Position", robot.getVerticalSlidePos());
                telemetry.addData("Upper Arm Position", robot.getUpperArmPos());
                telemetry.addData("Horizontal Slide Position", robot.getHorizontalSlidesPos());

                telemetry.addData("Max Drive Power", robot.maxDrivePower);
                telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
                telemetry.update();
            }
        }

//        telemetry.addAction()
        telemetry.speak("only in Ohio do Sigmas fanum tax skibidi gyatts");

        telemetry.addLine("wow");
        telemetry.update();
        sleep(5000);
    }

    public void initPositions() {

    }

    public enum TransferState {
        PICKUP,
        RETRACTING,
        RAISING_CLAW,
    }
    public TransferState transferState = TransferState.PICKUP;

    public void transferSample() {
        switch (transferState) {
            case PICKUP:
                break;
            case RETRACTING:
                break;

        }
    }

    public void prepareSpecimenPickup() {
        robot.setVerticalSlidesPos(JamalTwo.UPPER_SLIDES_PICKUP_SPECIMEN);

    }

    public void prepareSpecimenDeposit() {
        robot.setVerticalSlidesPos(JamalTwo.UPPER_SLIDES_ABOVE_HIGH_CHAMBER);
    }

    public void prepareSamplePickup() {

    }

    public void prepareSampleDeposit() {
        robot.setVerticalSlidesPos(JamalTwo.UPPER_SLIDES_DEPOSIT_SAMPLE);
    }

    ElapsedTime lowerSlideVeloTimer = new ElapsedTime();
    public void moveLowerSlides(Gamepad gamepad) {
        double input = gamepad.right_stick_x;
        double deltaTime = 0;
        double nextSlidePos = input * deltaTime * LOWER_SLIDE_MAX_SPEED;
        robot.setHorizontalSlidesPos(nextSlidePos);
    }

    public void moveUpperSlides(Gamepad gamepad) {
        double input = gamepad.left_stick_y;

        double slideVelo = input * UPPER_SLIDE_MAX_SPEED;
        robot.setVerticalSlidesVelocity(slideVelo);
    }

    public void moveDriveTrain(Gamepad gamepad) {

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
}
