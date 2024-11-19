package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class Thonk extends MecanumDrive {

    public Thonk(LinearOpMode opModeInstance) {
        super(opModeInstance);

        // 96mm Mecanum Wheels, 5203 Yellow Jacket 312 RPM
        dimensions = new RobotDimensions(
                18, 18, 18, 9.6, 537.68984
        );

        imu.initialize( new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));
    }
}
