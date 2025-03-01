package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalThree;

@Autonomous(name = "Park States Auto", group = "Autonomous") //alignment is the second tile end/third tile start
public class Park extends LinearOpMode {

    private JamalThree robot;
    private ElapsedTime runtime = new ElapsedTime();


    private double forwardDuration = 6.5;
    private double power = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JamalThree(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < forwardDuration)) {
            robot.drive(power, 0, 0);
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();
        }

        robot.drive(0, 0, 0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
