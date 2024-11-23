package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.CardboardOne;
import org.firstinspires.ftc.teamcode.robots.Inktonaut;


@Config
@Autonomous(name = "Blank Main Auto", group = "Main")
public class Blank extends LinearOpMode {
    CardboardOne robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CardboardOne(this);


        waitForStart();

        while(opModeIsActive()){



        }
    }
}
