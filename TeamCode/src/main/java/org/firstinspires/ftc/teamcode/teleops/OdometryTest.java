package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Odometry_Test", group = "Test")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode(){

        waitForStart();
        MotorEx motor1 = new MotorEx(hardwareMap, "left_encoder");

        while(opModeIsActive()){
            telemetry.addData("Position : ", motor1.getDistance());
            telemetry.update();
        }

    }

}
