package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.Outreach;
//
//@Config
@TeleOp(name = "Odometry Test", group = "Test")
public class OdoTests extends LinearOpMode {

    Outreach robot;

    @Override
    public void runOpMode(){

        robot = new Outreach(this);
        robot.maxDrivePower = 1.0;

        double nextX = 0;
        double nextY = 0;
        double nextHeading = 0;

        waitForStart();

        while (opModeIsActive()) {
            robot.updatePose();

            if (gamepad1.a && gamepad1.b) robot.resetPose();

            if (gamepad1.a)
                if      (gamepad1.right_bumper) nextX += 0.025;
                else if (gamepad1.left_bumper)  nextX -= 0.025;

            if (gamepad1.b)
                if      (gamepad1.right_bumper) nextY += 0.025;
                else if (gamepad1.left_bumper)  nextY -= 0.025;

            if (gamepad1.y)
                if      (gamepad1.right_bumper) nextHeading += 0.1;
                else if (gamepad1.left_bumper)  nextHeading -= 0.1;
            nextHeading = Range.clip(nextHeading, -180, 180);

            if (gamepad1.x)
                robot.moveBy(new Pose2d(nextX, nextY, new Rotation2d(Math.toRadians(nextHeading))), 2);



            if (gamepad1.dpad_up) {
                robot.moveBy(10, 0, 0, 2);
//                robot.moveBy(new Pose2d(10, 0, new Rotation2d(0)), 2);

            } else if (gamepad1.dpad_right) {
                robot.moveBy(-10,0,0,2);
//                robot.moveBy(new Pose2d(-10, 0, new Rotation2d(0)), 2);

            } else if (gamepad1.dpad_down) {
                robot.moveBy(0,0,90,2);
//                robot.moveBy(new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))), 2);

            } else if (gamepad1.dpad_left) {
                robot.moveBy(10,0,90,2);
//                robot.moveBy(new Pose2d(10, 10, new Rotation2d(Math.toRadians(90))), 2);
            }

            telemetry.addData("Pose (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", robot.currentPose.getX(), robot.currentPose.getY(), robot.currentPose.getRotation().getDegrees());
            telemetry.addData("Next Target (x,y,heading)", "(%5.2f, %5.2f, %6.2f)", nextX, nextY, nextHeading);
            telemetry.update();
        }
    }

}
