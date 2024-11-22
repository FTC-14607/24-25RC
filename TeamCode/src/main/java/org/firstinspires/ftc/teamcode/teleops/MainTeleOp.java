package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        double verticalSlidePos = CardboardOne.VERT_SLIDES_BOTTOM;

//        robot.closeSampleClaw();
//        robot.raiseSampleClaw();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            // intake
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.closeSampleClaw();
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.openSampleClaw();
            }

            // outtake
            if (gamepad2.a) {
                robot.dumpDumper();
                sleep(10);
                robot.resetDumper();
            }

            // horizontal slides
            if (gamepad2.dpad_right) {
                horizontalSlidePos += 0.005;
            } else if (gamepad2.dpad_left) {
                horizontalSlidePos -= 0.005;
            }
            horizontalSlidePos = Range.clip(horizontalSlidePos, CardboardOne.HORI_SLIDES_EXTENDED, CardboardOne.HORI_SLIDES_RETRACTED);
            robot.setHorizontalSlidesPos(horizontalSlidePos);

            //

            if (gamepad2.a) {
                if (gamepad2.right_bumper) robot.extendHorizontalSlides();
                else if (gamepad2.left_bumper) robot.retractHorizontalSlides();
            }

            else if (gamepad2.b) {
                if (gamepad2.right_bumper) robot.raiseArm();
                else if (gamepad2.left_bumper) robot.lowerArm();
            }

            moveDriveTrain(gamepad1);

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

    public void moveDriveTrain(Gamepad gamepad) {

        // TODO: field centric

        // change max drive power
        if      (gamepad.a) robot.maxDrivePower = 0.3;
        else if (gamepad.b) robot.maxDrivePower = 0.5;
        else if (gamepad.y) robot.maxDrivePower = 0.7;
        else if (gamepad.x) robot.maxDrivePower = 0.9;

        // set drivetrain power
        double throttle = gamepad.left_stick_y * -1;
        double strafe   = gamepad.left_stick_x;
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
