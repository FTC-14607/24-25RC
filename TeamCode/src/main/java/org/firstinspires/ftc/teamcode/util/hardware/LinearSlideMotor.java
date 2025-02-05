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
 * PID loop.
 */
public class LinearSlideMotor extends PIDFMotor {

    public LinearSlideMotor(DcMotorEx motor) {
        super(motor);
    }

    /**
     * Slides use output = gain * input Feedforward function since the higher the slide goes, the
     * more mass the motors are lifting, the more force required, the more voltage required. The
     * relationship is directly proportional because F_gravity = mg
     */
    @Override
    public double getFeedforwardOutput(int currentPosition) {
        return velocityFeedforwardGain * currentPosition;
    }
}
