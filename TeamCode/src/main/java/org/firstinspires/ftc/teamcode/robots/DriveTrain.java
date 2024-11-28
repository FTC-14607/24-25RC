package org.firstinspires.ftc.teamcode.robots;

public interface DriveTrain {

    double[] drive(double throttle, double strafe, double rotate);
    void brake();
}
