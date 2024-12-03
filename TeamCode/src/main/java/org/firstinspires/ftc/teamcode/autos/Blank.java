package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Jamal;


@Config
@Autonomous(name = "Blank Main Auto", group = "Main", preselectTeleOp = "Main TeleOp")
public class Blank extends LinearOpMode {
    Jamal robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Jamal(this);


        waitForStart();

        while(opModeIsActive()){



        }
    }
}
