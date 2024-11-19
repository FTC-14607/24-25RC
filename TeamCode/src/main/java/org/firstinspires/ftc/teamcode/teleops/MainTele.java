package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.robots.Thonk;

@TeleOp(name = "Main Linear TeleOp", group = "Main")
public class MainTele extends LinearOpMode {

    Thonk robot;
    ElapsedTime loopTimer = new ElapsedTime();

    public enum ArmState {
        RETRACTED,
        EXTENDING_HORIZONTAL,
        EXTENDING_VERTICAL
    }
    public ArmState armState = ArmState.RETRACTED;

    public static final double STRAIGHT_CORRECTION = 0.0;

    public void runOpMode() {
        robot = new Thonk(this);
        robot.maxDrivePower = 0.9;

        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            loopTimer.reset();

            switch (armState) {
                case RETRACTED:
                    break;
                case EXTENDING_HORIZONTAL:
                    break;
                case EXTENDING_VERTICAL:
                    break;
            }



            if (gamepad1.a)
                robot.maxDrivePower = 0.3;
            else if (gamepad1.b)
                robot.maxDrivePower = 0.5;
            else if (gamepad1.y)
                robot.maxDrivePower = 0.7;
            else if (gamepad1.y)
                robot.maxDrivePower = 0.9;

            moveDriveTrain(gamepad1);

            telemetry.addData("Max Drive Power", robot.maxDrivePower);
            telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
            telemetry.update();
        }
    }

    public void moveDriveTrain(Gamepad gamepad) {
        double throttle = gamepad.left_stick_y;
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
