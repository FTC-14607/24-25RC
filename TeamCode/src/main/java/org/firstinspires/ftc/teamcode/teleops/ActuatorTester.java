package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@TeleOp(name = "Acuatator Tester", group = "Test")
public class ActuatorTester extends LinearOpMode {

    JamalTwo robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public enum ArmState {
        RETRACTED,
        EXTENDING_HORIZONTAL,
        EXTENDING_VERTICAL
    }

    public ArmState armState = ArmState.RETRACTED;

    public static final double STRAIGHT_CORRECTION = 0.0;

    public void runOpMode() {
        robot = new JamalTwo(this);
        double servoPos = 0;
        robot.maxDrivePower = 0.9;
        int slidePos = 0;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            if      (gamepad2.right_trigger > 0) servoPos = Math.min(1, servoPos + 0.0002);
            else if (gamepad2.left_trigger  > 0) servoPos = Math.max(0, servoPos - 0.0002);

            if (gamepad2.a) {
                if (gamepad2.dpad_down) {
                    JamalTwo.setRunMode(robot.upperSlides, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else if (gamepad2.right_trigger > 0) {
                    slidePos += 1;
                } else if (gamepad2.left_trigger > 0) {
                    slidePos = Math.max(0, slidePos - 1);
                }

                robot.setUpperSlidesPos(slidePos);
            }

            else if (gamepad2.b) {
                if (gamepad2.dpad_up) {
                    robot.lowerSlideRight.setPosition(servoPos);
                } else if (gamepad2.dpad_right) {
                    robot.lowerSlideLeft.setPosition(servoPos);
                } else if (gamepad2.dpad_left) {
                    robot.setLowerSlidesPos(servoPos);
                }

            }

            else if (gamepad2.y) {
                if (gamepad2.dpad_up) {
                    robot.upperClaw.setPosition(servoPos);
                } else if (gamepad2.dpad_left) {
                    robot.upperArmRight.setPosition(servoPos);
                } else if (gamepad2.dpad_down) {
                    robot.upperArmLeft.setPosition(servoPos);
                } else if (gamepad2.dpad_right) {
                    robot.upperClawPitch.setPosition(servoPos);
                }
            }

            else if (gamepad2.x) {

                if (gamepad2.dpad_up) {
                    robot.lowerClaw.setPosition(servoPos);
                } else if (gamepad2.dpad_left) {
                    robot.lowerClawYaw.setPosition(servoPos);
                } else if (gamepad2.dpad_down) {
                    robot.lowerClawPitch.setPosition(servoPos);
                }
            }

            moveDriveTrain(gamepad1);

            telemetry.addData("servoPos", servoPos);
            telemetry.addData("slidePos", slidePos);

            telemetry.addData("Vertical Slide Position", robot.getUpperSlidesPos());
            telemetry.addData("Horizontal Slide Position", robot.getLowerSlidesPos());
            telemetry.addData("Upper Arm Position", robot.getUpperArmPos());
            telemetry.addData("Upper Claw Position", robot.getUpperClawPos());
            telemetry.addData("Upper Claw Pitch Position", robot.getUpperClawPitchPos());
            telemetry.addData("Lower Claw Position", robot.getLowerClawPos());
            telemetry.addData("Lower Claw Yaw Posiition", robot.getLowerClawYawPos());
            telemetry.addData("Lower Claw Pitch Position", robot.getLowerClawPitchPos());

            telemetry.addData("Max Drive Power", robot.maxDrivePower);
            telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
            telemetry.update();
        }
    }

    public void moveDriveTrain(Gamepad gamepad) {

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

        // due to mechanical imperfection, robot won't necessarily drive straight
        if (throttle != 0)
            rotate += STRAIGHT_CORRECTION * ((throttle > 0) ? 1 : -1);

        robot.drive(throttle, strafe, rotate);
    }
}
