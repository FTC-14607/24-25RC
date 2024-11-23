package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.CardboardOne;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    CardboardOne robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public enum ArmState {
        RETRACTED,
        EXTENDING_HORIZONTAL,
        EXTENDING_VERTICAL
    }

    public ArmState armState = ArmState.RETRACTED;

    public static final double STRAIGHT_CORRECTION = 0.0;

    public void runOpMode() {
        robot = new CardboardOne(this);
        robot.maxDrivePower = 0.9;

        double horizontalSlidePos = CardboardOne.HORI_SLIDES_RETRACTED;
        int verticalSlidePos = CardboardOne.VERT_SLIDES_BOTTOM;


        robot.raiseSampleClaw();
        robot.setSampleClawYawPos(CardboardOne.SAMPLE_CLAW_HORIZONTAL);
        robot.closeSampleClaw();
        robot.retractHorizontalSlides();
//        robot.openSpecimenClaw();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

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
//            horizontalSlidePos = Range.clip(horizontalSlidePos, CardboardOne.HORI_SLIDES_EXTENDED, CardboardOne.HORI_SLIDES_RETRACTED);
//            robot.setHorizontalSlidesPos(horizontalSlidePos);

            else if (gamepad2.b && gamepad2.x) {
                if (gamepad2.right_bumper) robot.raiseArm();
                else if (gamepad2.left_bumper) robot.lowerArm();
            }

            // vertical slides
            if (gamepad2.right_trigger > 0) verticalSlidePos += 10;
            else if (gamepad2.left_trigger > 0) verticalSlidePos -= 10;
            if (gamepad2.x) verticalSlidePos = CardboardOne.VERT_SLIDES_ABOVE_TOP_TRUSS;
            else if (gamepad2.y) verticalSlidePos = CardboardOne.VERT_SLIDES_BELOW_TOP_TRUSS;
            else if (gamepad2.left_stick_button) verticalSlidePos = CardboardOne.VERT_SLIDES_PICKUP_SPECIMEN;

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
