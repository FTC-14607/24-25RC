package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.JamalOne;


@Config
@Autonomous(name = "Blank Main Auto", group = "Main", preselectTeleOp = "Main TeleOp")
public class Blank extends LinearOpMode {
//    JamalOne robot;

    @Override
    public void runOpMode() throws InterruptedException {
//        robot = new JamalOne(this);
        waitForStart();
        while(opModeIsActive()){

        }
    }
}
