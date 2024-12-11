package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@TeleOp(name = "Slide Tester", group = "Main")
public class SlideTester extends LinearOpMode {

    JamalTwo robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public void runOpMode() {
        robot = new JamalTwo(this);

        int slidePos = 0;
        double slideVelo = 0;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

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
                robot.setVerticalSlidesPos(slidePos);

            else if (gamepad1.b)
                robot.setVerticalSlidesVelocity(slideVelo);

            telemetry.addData("slidePos", slidePos);

            telemetry.addData("Vertical Slide Position", robot.getVerticalSlidePos());
            telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
            telemetry.update();
        }
    }

}
