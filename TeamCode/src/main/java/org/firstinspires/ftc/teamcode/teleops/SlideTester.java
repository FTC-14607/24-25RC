package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalTwo;
import org.firstinspires.ftc.teamcode.util.hardware.LinearSlideMotor;

@Config
@TeleOp(name = "Slide Tester", group = "Test")
public class SlideTester extends LinearOpMode {

    JamalTwo robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public void runOpMode() {
        robot = new JamalTwo(this);

        double slidePos = 0;
        double slideVelo = 0;
        double p = 0, i = 0, d = 0, f = 0;
        PIDFCoefficients veloPid = new PIDFCoefficients(1,0,0,0);

        telemetry.addData("Voltage", robot.controlHubVoltage);
        telemetry.addData("voltage scale", robot.voltageScaler);
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();
            robot.update();

            if (gamepad1.right_trigger > 0) {
                f += 0.0000001;
                for (LinearSlideMotor slide : robot.upperSlides)
                    slide.addF(f);
            } else if (gamepad1.left_trigger > 0) {
                f -= 0.0000001;
                for (LinearSlideMotor slide : robot.upperSlides)
                    slide.addF(f);
            }
            else if (gamepad1.left_stick_button) {
                for (DcMotorEx slide : robot.upperSlides)
                    slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, veloPid);
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                JamalTwo.setRunMode(robot.upperSlides, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                JamalTwo.setRunMode(robot.upperSlides, DcMotor.RunMode.RUN_TO_POSITION);
            }

            if      (gamepad1.dpad_up)
                slidePos = Math.min(slidePos + 0.5, JamalTwo.UPPER_SLIDES_TOP);
            else if (gamepad1.dpad_down)
                slidePos = Math.max(slidePos - 0.5, JamalTwo.UPPER_SLIDES_BOTTOM);

            else if (gamepad1.dpad_right)
                slideVelo += 0.1;
            else if (gamepad1.dpad_left)
                slideVelo -= 0.1;

            if      (gamepad1.a)
                robot.setUpperSlidesVelocity(0);

            else if (gamepad1.b) {
                robot.upperSlideRight.setPower(0.6);
                robot.upperSlideLeft.setPower(0.6);
//                robot.setUpperSlidesVelocity(slideVelo);
            }
            else if (gamepad1.x) {
                robot.upperSlideRight.setPower(-0.6);
                robot.upperSlideLeft.setPower(-0.6);
//                robot.setUpperSlidesVelocity(0);
            }
            else if (gamepad1.y)
                robot.setUpperSlidesPos(2000);

            telemetry.addData("slidePos", slidePos);
            telemetry.addData("slideVelo", slideVelo);
            telemetry.addData("f", f);
            telemetry.addData("slidepower r", robot.upperSlideRight.getPower());
            telemetry.addData("slidepower l", robot.upperSlideLeft.getPower());

            telemetry.addData("Vertical Slide Position", robot.getUpperSlidesPos());
            telemetry.addData("pos pid", robot.upperSlideRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("velo pid", robot.upperSlideRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

            telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
            telemetry.update();
        }
    }

}
