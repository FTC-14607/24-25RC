package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.CardboardOne;


@Config
@Autonomous(name = "Park Main Auto", group = "Main")
public class Park extends LinearOpMode {
    CardboardOne robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CardboardOne(this);


        waitForStart();


        robot.drive(0.7,0,0);
        sleep(600);

        robot.drive(0,0,0);

    }
}
