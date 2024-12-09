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
    ElapsedTime loopTimer = new ElapsedTime();

    public enum LowerSystemState {
        INIT,
        TRANSFERRING,
        INTAKING_SAMPLE,
    }

    public enum UpperSystemState {
        INIT,
        TRANSFERRING,
        INTAKING_SPECIMEN,
        OUTTAKING_SAMPLE,
        OUTTAKING_SPECIMEN,
        ASCENDING,
    }

    public LowerSystemState lowerSystemState = LowerSystemState.INIT;
    public UpperSystemState upperSystemState = UpperSystemState.INIT;

    public ElapsedTime lowerTimer = new ElapsedTime();
    public ElapsedTime upperTimer = new ElapsedTime();

    public static final double STRAIGHT_CORRECTION = 0.0;

    public void runOpMode() {
        robot = new JamalTwo(this);
        robot.maxDrivePower = 0.9;

        double horizontalSlidePos = JamalTwo.LOWER_SLIDES_RETRACTED;
        int verticalSlidePos = JamalTwo.UPPER_SLIDES_BOTTOM;


//        robot.raiseSampleClaw();
//        robot.setSampleClawYawPos(JamalOne.SAMPLE_CLAW_HORIZONTAL);
//        robot.closeSampleClaw();
//        robot.retractHorizontalSlides();
//        robot.openSpecimenClaw();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            switch (lowerSystemState) {
                case INIT:
                    break;
                case TRANSFERRING:
                    break;
            }

            switch (upperSystemState) {

            }
            // TODO: univeral robot.update() method


            // TODO: reorganize controls into methods
            // TODO: add finite state machine so things don't break
            // intake
            if (gamepad1.right_bumper || ( (!(gamepad2.a || gamepad2.b) && gamepad2.right_bumper))) {
                robot.closeSampleClaw();
            } else if (gamepad1.left_bumper || (!(gamepad2.a || gamepad2.b) && gamepad2.left_bumper)) {
                robot.openSampleClaw();
            }

            if (gamepad2.b) {
                if (gamepad2.right_bumper) robot.raiseSampleClaw();
                else if (gamepad2.left_bumper) robot.lowerSampleClaw();
            }

            // outtake
            if (gamepad2.a && gamepad2.b) {
                robot.dumpDumper();
                sleep(10);
                robot.resetDumper();
            }
            if (gamepad2.dpad_up) robot.closeSpecimenClaw();
            else if (gamepad2.dpad_down) robot.openSpecimenClaw();

            // horizontal slides
            if (gamepad2.a) {
                if (gamepad2.right_bumper) robot.extendHorizontalSlides();
                else if (gamepad2.left_bumper) robot.retractHorizontalSlides();
            }

//            if (gamepad2.dpad_right) {
//                horizontalSlidePos += 0.005;
//            } else if (gamepad2.dpad_left) {
//                horizontalSlidePos -= 0.005;
//            }
//            horizontalSlidePos = Range.clip(horizontalSlidePos, Jamal.HORI_SLIDES_EXTENDED, Jamal.HORI_SLIDES_RETRACTED);
//            robot.setHorizontalSlidesPos(horizontalSlidePos);

            else if (gamepad2.b && gamepad2.x) {
                if (gamepad2.right_bumper) robot.raiseArm();
                else if (gamepad2.left_bumper) robot.lowerArm();
            }

            // vertical slides
            if (gamepad2.right_trigger > 0) verticalSlidePos += 10;
            else if (gamepad2.left_trigger > 0) verticalSlidePos -= 10;
            if (gamepad2.x) verticalSlidePos = JamalOne.VERT_SLIDES_ABOVE_TOP_TRUSS;
            else if (gamepad2.y) verticalSlidePos = JamalOne.VERT_SLIDES_BELOW_TOP_TRUSS;
            else if (gamepad2.left_stick_button) verticalSlidePos = JamalOne.VERT_SLIDES_PICKUP_SPECIMEN;

            robot.setVerticalSlidesPos(verticalSlidePos);

            moveDriveTrain(gamepad1);


            if (true) {
                telemetry.addData("Vertical Slide Position", robot.getVerticalSlidePos());
                telemetry.addData("Upper Arm Position", robot.getUpperArmPos());
                telemetry.addData("Dumper Position", robot.getDumperPos());
                telemetry.addData("Horizontal Slide Position", robot.getHorizontalSlidesPos());
                telemetry.addData("Sample Claw Position", robot.getSampleClawPos());
                telemetry.addData("Sample Claw Yaw Posiition", robot.getSampleClawYawPos());
                telemetry.addData("Sample Claw Pitch Position", robot.getSampleClawPitchPos());
                telemetry.addData("Specimen Claw Position", robot.getSpecimenClawPos());

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

    public void moveLowerSlides() {

    }

    public void moveUpperSlides() {

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

        // TODO: get this
        // due to mechanical imperfection, robot won't necessarily drive straight
        if (throttle != 0)
            rotate += STRAIGHT_CORRECTION * ((throttle > 0) ? 1 : -1);
        else if (strafe != 0)
            rotate += STRAIGHT_CORRECTION * ((strafe > 0) ? -1 : 1);

        robot.drive(throttle, strafe, rotate);
    }
}
