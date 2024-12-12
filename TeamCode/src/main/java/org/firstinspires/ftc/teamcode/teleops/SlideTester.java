package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@TeleOp(name = "Slide Tester", group = "Test")
public class SlideTester extends LinearOpMode {

    JamalTwo robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public void runOpMode() {
        robot = new JamalTwo(this);

        int slidePos = 0;
        double slideVelo = 0;
        double p = 0, i = 0, d = 0, f = 0;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            if (gamepad1.right_trigger > 0) {
                f += 0.0001;
                for (DcMotorEx slide : robot.upperSlides) {
                    slide.setVelocityPIDFCoefficients(p, i, d, f);
                }
            } else if (gamepad1.left_trigger > 0) {
                f -= 0.0001;
                for (DcMotorEx slide : robot.upperSlides) {
                    slide.setVelocityPIDFCoefficients(p, i, d, f);
                }
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                JamalTwo.setRunMode(robot.upperSlides, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                JamalTwo.setRunMode(robot.upperSlides, DcMotor.RunMode.RUN_TO_POSITION);
            }

            if      (gamepad1.dpad_up)
                slidePos = Math.min(slidePos + 5, JamalTwo.UPPER_SLIDES_TOP);
            else if (gamepad1.dpad_down)
                slidePos = Math.max(slidePos - 5, JamalTwo.UPPER_SLIDES_BOTTOM);

            else if (gamepad1.dpad_right)
                slideVelo += 1;
            else if (gamepad1.dpad_left)
                slideVelo -= 1;

            if      (gamepad1.a)
                robot.setUpperSlidesPos(slidePos);

            else if (gamepad1.b)
                robot.setUpperSlidesVelocity(slideVelo);

            telemetry.addData("slidePos", slidePos);
            telemetry.addData("slideVelo", slideVelo);

            telemetry.addData("Vertical Slide Position", robot.getUpperSlidesPos());
            telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
            telemetry.update();
        }
    }

}
