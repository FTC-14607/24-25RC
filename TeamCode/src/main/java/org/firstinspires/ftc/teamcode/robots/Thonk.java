package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class Thonk extends MecanumDrive {

    public Thonk(LinearOpMode opModeInstance) {
        super(opModeInstance);

        imu.initialize( new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        ));
    }
}
