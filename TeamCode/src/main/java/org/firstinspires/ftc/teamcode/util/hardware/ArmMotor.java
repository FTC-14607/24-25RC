package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * A motor for a rotational arm fixed on an unmoving axle.
 */
public class ArmMotor extends PIDFMotor {

    private double ticksPerRotation;
    private int horizontalPosition;

    /**
     *
     * @param motor from hardwareMap.get
     * @param ticksPerRotation output shaft ticks per rotation
     * @param horizontalPosition tick position where arm is horizontal
     */
    public ArmMotor(DcMotorEx motor, double ticksPerRotation, int horizontalPosition) {
        super(motor);
        this.ticksPerRotation = ticksPerRotation;
        this.horizontalPosition = horizontalPosition;
    }

    /**
     * Feedforward function models the changing torque from gravity as the arm rotates. This ends
     * up being the cosine of the angle of the arm from horizontal, and is negated to counteract
     * this force.
     * @param position
     * @return
     */
    @Override
    public double getFeedforwardOutput(int position) {
        position -= horizontalPosition;
        double rotations = position / ticksPerRotation;
        double angle = 2 * Math.PI * rotations;

        return velocityFeedforwardGain * -Math.cos(angle);
    }

}
