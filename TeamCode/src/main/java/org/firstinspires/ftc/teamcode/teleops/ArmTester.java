package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalThree;
import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@TeleOp(name = "Arm Tester", group = "Test")
public class ArmTester extends LinearOpMode {
    public static final double STRAIGHT_CORRECTION = 0.0;

    JamalThree robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public void runOpMode() {
        robot = new JamalThree(this);

        double motorPos = 0;
        double feedf = 0;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();
            robot.update();

            if      (gamepad2.right_trigger > 0) motorPos = Math.min(6000, motorPos + 1);
            else if (gamepad2.left_trigger  > 0) motorPos = Math.max(-600, motorPos - 1);

            if      (gamepad2.right_bumper) feedf += 0.0001;
            else if (gamepad2.left_bumper)  feedf -= 0.0001;

            robot.upperArm.addF(feedf);

            if (gamepad2.a) {
                if (gamepad2.dpad_down) {
                    robot.upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else if (gamepad2.right_trigger > 0) {
                    robot.upperArm.setPowerEx(0.3);
                } else if (gamepad2.left_trigger > 0) {
                    robot.upperArm.setPowerEx(-0.3);
                } else {
                    robot.upperArm.brake();
                }
            }


            else if (gamepad2.b) {
                if (gamepad2.dpad_up) {
                    robot.setUpperArmPos((int)motorPos);
                } else if (gamepad2.dpad_right) {
                } else if (gamepad2.dpad_left) {
                }

            }

            else if (gamepad2.y) {
                if (gamepad2.dpad_up) {
                    robot.setUpperArmVelocity(200);
                } else if (gamepad2.dpad_down) {
                    robot.setUpperArmVelocity(0);
                } else if (gamepad2.dpad_right) {
                }
            }

            else if (gamepad2.x) {
                if (gamepad2.dpad_up) {
                } else if (gamepad2.dpad_left) {
                } else if (gamepad2.dpad_down) {
                }
            }

            moveDriveTrain(gamepad1);

            telemetry.addData("motorPos", motorPos);
            telemetry.addData("feedf", feedf);
            telemetry.addData("Upper Arm Position", robot.getUpperArmPos());

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
        double strafe   = gamepad.left_stick_x * -1;
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
