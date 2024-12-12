package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blank Main Auto", group = "Main", preselectTeleOp = "Main TeleOp")
public class Blank extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Running blank auto...");
            telemetry.update();
        }
    }
}
