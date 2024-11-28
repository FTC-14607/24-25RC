package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.Outreach;
import org.firstinspires.ftc.teamcode.util.odometry.OdometryDriver;

@Config
@TeleOp(name = "Odometry Test 2", group = "Test")
public class OdoTests2 extends LinearOpMode {

    Outreach robot;

    @Override
    public void runOpMode(){

        robot = new Outreach(this);
        robot.maxDrivePower = 0.9;
        OdometryDriver odoDriver = new OdometryDriver(robot.odo, robot, this);

        Pose2d target = new Pose2d(0,0, new Rotation2d(0));
        odoDriver.setTarget(target);

        waitForStart();

        while (opModeIsActive()) {
            odoDriver.updatePose();

            if (gamepad1.a && gamepad1.b) odoDriver.resetPose();

            if (gamepad1.atRest())
                odoDriver.driveToTarget(0);
            else
                moveDriveTrain(gamepad1);

            telemetry.addData("Pose (x,y,heading)",
                    "(%5.2f, %5.2f, %6.2f)", odoDriver.currentPose.getX(), odoDriver.currentPose.getY(), odoDriver.currentPose.getRotation().getDegrees());
            telemetry.update();
        }
    }

    public void moveDriveTrain(Gamepad gamepad) {
        double throttle = gamepad.left_stick_y * -1;
        double strafe   = gamepad.left_stick_x;
        double rotate   = gamepad.right_stick_x;

        robot.drive(throttle, strafe, rotate);
    }

}
