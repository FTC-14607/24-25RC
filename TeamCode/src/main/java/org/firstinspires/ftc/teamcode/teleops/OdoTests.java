package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Outreach;

@TeleOp(name = "Odometry Test", group = "Test")
public class OdoTests extends LinearOpMode {

    Outreach robot;

    @Override
    public void runOpMode(){

        robot = new Outreach(this);
        robot.maxDrivePower = 0.7;

        waitForStart();

        robot.forward(30, 4);
        robot.forward(-20, 4);
        robot.forward(10, 4);

    }

}
