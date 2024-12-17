package org.firstinspires.ftc.teamcode.util.hardware;

import android.sax.StartElementListener;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

/**
 * A motor for strung, linear slide systems with an additional external
 * PID loop. This circumvents the built-in update speed cap of 50ms, but
 * it should be noted that the reason that cap exists is because the extra
 * speed is usually unnecessary CPU load. This also lets you use the
 * feedforward term properly.
 */
public class LinearSlideMotor extends DcMotorImplEx {

    private int currentPosition;
    private double currentVelocity;

    private PIDController positionController;         // input: ticks     output: ticks/sec
    private PIDController velocityFeedbackController; // input: ticks/sec output: motor power
    private double velocityFeedforwardGain;           // input: ticks     output: motor power

    private SlideState state = SlideState.INACTIVE;
    private double ticksPerInch;                      // input: inches    output: ticks
    private double voltageScaler = 1.0;
    public double maxPower = 1.0;

    public int bottomPosition; // ticks
    public int topPosition;

    public enum SlideState {
        INACTIVE,
        BRAKE,
        RUN_TO_POSITION,
        RUN_AT_VELOCITY
    }

    public LinearSlideMotor(DcMotorEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    /**
     *
     * @param bottom [ticks]
     * @param top [ticks]
     * @param maxPower [0-1]
     * @param posP
     * @param veloP
     * @param veloI
     * @param veloD
     * @param F
     */
    public void addControl(int bottom, int top, double voltageScaler, double maxPower, double posP, double veloP, double veloI, double veloD, double F) {
        bottomPosition = bottom;
        topPosition = top;
        this.voltageScaler = voltageScaler;
        this.maxPower = maxPower;
        positionController = new PIDController(posP, 0, 0);
        velocityFeedbackController = new PIDController(veloP, veloI, veloD);
        velocityFeedforwardGain = F;
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    public void addF(double f) {
        velocityFeedforwardGain = f;
    }

    public double ffpower = 0, fbpower = 0, totpower = 0;
    public RunMode rmode = null;
    public void update() {
        currentPosition = getCurrentPosition();
        currentVelocity = getVelocity();

        switch (state) {
            case INACTIVE:
                break;

            case BRAKE:
                setPower(0);
                break;

            case RUN_TO_POSITION:
                velocityFeedbackController.setSetPoint(
                        positionController.calculate(currentPosition)
                );
                // flows to velocity case

            case RUN_AT_VELOCITY:
                double power = velocityFeedforwardGain * currentPosition;

                ffpower = power;

                // prevent changing velocity when near the bounds
                double feedBackPower = velocityFeedbackController.calculate(currentVelocity);
                if (
                        currentPosition <= bottomPosition && feedBackPower < 0 ||
                        currentPosition >= topPosition    && feedBackPower > 0
                ) {
                    feedBackPower = 0;
                }
                power += feedBackPower;
                fbpower = feedBackPower;
                power *= voltageScaler;
                rmode = getMode();
                power = Range.clip(power, -maxPower, maxPower);


                totpower = power;
                setPower(power);
                break;
        }
    }

    public int getLastPosition() {
        return currentPosition;
    }
    public double getLastVelocity() {
        return currentVelocity;
    }

    public int getTargetPositionEx() {
        return (int) positionController.getSetPoint();
    }

    public double getTargetVelocityEx() {
        return velocityFeedbackController.getSetPoint();
    }

    public void pauseControl() {
        state = SlideState.INACTIVE;
    }

    public void brake() {
        state = SlideState.BRAKE;
    }

    public void setPowerEx(double power) {
        pauseControl();
        if (
                currentPosition < bottomPosition*1.01 && power < 0 ||
                currentPosition > topPosition   *0.99 && power > 0
        ) {
            brake();
            return;
        }
        setPower(power);
    }

    public void setTargetPositionEx(int targetPosition) {
        targetPosition = Range.clip(targetPosition, bottomPosition, topPosition);
        positionController.setSetPoint(targetPosition);
        state = SlideState.RUN_TO_POSITION;
    }

    public void setTargetPositionInches(double targetPositionInches) {
        setTargetPositionEx((int) (targetPositionInches * ticksPerInch));
    }

    public void setTargetVelocityEx(double targetVelocity) {
        velocityFeedbackController.setSetPoint(targetVelocity);
        state = SlideState.RUN_AT_VELOCITY;
    }

}
