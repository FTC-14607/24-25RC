package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.JamalOne;

@Autonomous(name = "Park Main Auto", group = "Main")
public class Park extends LinearOpMode {
    JamalOne robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JamalOne(this);

        waitForStart();

        robot.drive(0.6,0,0);
        sleep(2000);
        robot.brake();
    }
}

