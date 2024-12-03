package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.Jamal;

@TeleOp(name = "Acuatator Tester", group = "Main")
public class ActuatorTester extends LinearOpMode {

    Jamal robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public enum ArmState {
        RETRACTED,
        EXTENDING_HORIZONTAL,
        EXTENDING_VERTICAL
    }

    public ArmState armState = ArmState.RETRACTED;

    public static final double STRAIGHT_CORRECTION = 0.0;

    public void runOpMode() {
        robot = new Jamal(this);
        double servoPos = 0;
        robot.maxDrivePower = 0.9;
        int slidePos = 0;

//        robot.closeSampleClaw();
//        robot.raiseSampleClaw();
//        robot.lowerArm();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            if (gamepad2.right_trigger > 0) servoPos += 0.001;
            else if (gamepad2.left_trigger > 0) servoPos -= 0.001;

            if (gamepad2.a) {
                if (gamepad2.dpad_down) {
                    robot.setRunMode(robot.vertSlides, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else if (gamepad2.right_trigger > 0) {
                    slidePos += 1;
                } else if (gamepad2.left_trigger > 0) {
                    slidePos -= 1;
                }

                robot.setVerticalSlidesPos(slidePos);
            }

            else if (gamepad2.b) {
                if (gamepad2.dpad_up) {
                    robot.horiSlideRight.setPosition(servoPos);
                } else if (gamepad2.dpad_right) {
                    robot.horiSlideLeft.setPosition(servoPos);
                } else if (gamepad2.dpad_left) {
                    robot.setHorizontalSlidesPos(servoPos);
                }

            }

            else if (gamepad2.y) {
                if (gamepad2.dpad_up) {

                } else if (gamepad2.dpad_left) {
                    robot.setUpperArmPos(servoPos);
                }
            }

            else if (gamepad2.x) {
                if (gamepad2.dpad_up) {

                } else if (gamepad2.dpad_left) {
                    robot.setDumperPos(servoPos);
                }
            }

            else if (gamepad2.right_bumper) {

                if (gamepad2.dpad_up) {
                    robot.setSampleClawPos(servoPos);
                } else if (gamepad2.dpad_left) {
                    robot.setSampleClawYawPos(servoPos);
                } else if (gamepad2.dpad_down) {
                    robot.setSampleClawPitchPos(servoPos);
                }
            }

            else if (gamepad2.left_bumper) {
                if (gamepad2.dpad_left) {
                    robot.setSpecimenClawPos(servoPos);
                }
            }

            moveDriveTrain(gamepad1);

            telemetry.addData("servoPos", servoPos);

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
