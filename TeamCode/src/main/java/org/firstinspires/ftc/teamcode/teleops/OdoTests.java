package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Outreach;

@Config
@TeleOp(name = "Odometry Test", group = "Test")
public class OdoTests extends LinearOpMode {

    Outreach robot;

    @Override
    public void runOpMode(){

        robot = new Outreach(this);
        robot.maxDrivePower = 0.7;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.forward(30, 4);
                robot.forward(-20, 4);
                robot.forward(10, 4);
            } else if (gamepad1.b) {
                robot.rotate(90, 4);
                return;
            } else if (gamepad1.x) {
                robot.strafe(20, 4);
            } else if (gamepad1.y) {
                robot.resetPose();
            }

            if (gamepad1.dpad_up) {
                robot.moveBy(new Pose2d(10, 0, new Rotation2d(0)), 4);

            } else if (gamepad1.dpad_right) {
                robot.moveBy(new Pose2d(0, 10, new Rotation2d(0)), 4);

            } else if (gamepad1.dpad_down) {
                robot.moveBy(new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))), 4);

            } else if (gamepad1.dpad_left) {
                robot.moveBy(new Pose2d(10, 10, new Rotation2d(Math.toRadians(90))), 4);
            }

            robot.updatePose();
            telemetry.addData("Pose", Outreach.currentPose);
            telemetry.update();
        }
    }

}
