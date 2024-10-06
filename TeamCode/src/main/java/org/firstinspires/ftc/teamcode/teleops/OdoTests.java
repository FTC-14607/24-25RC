package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robots.Outreach;
import org.firstinspires.ftc.teamcode.util.odometry.FTCLibOdometry;

@TeleOp(name = "Odometry Test", group = "Test")
public class OdoTests extends LinearOpMode {

    Outreach robot;

    @Override
    public void runOpMode(){

        robot = new Outreach(this);
        robot.odo.init();

        waitForStart();
        while(opModeIsActive()){

            robot.odo.update();

            robot.forwardOdo(10);

            telemetry.addData("Pose", robot.odo.getPose());
            telemetry.update();
            stop();
        }

    }

}
