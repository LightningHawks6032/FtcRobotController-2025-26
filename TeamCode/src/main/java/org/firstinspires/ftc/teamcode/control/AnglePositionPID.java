package org.firstinspires.ftc.teamcode.control;

import static java.lang.Math.exp;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Util;

public class AnglePositionPID {
    float accumulatedError;
    PIDF.Weights coeff;

    public float loop(float cx, float tx, float cv, float dt) {
        float error = Util.normAngle(tx - cx);

        accumulatedError *= (float)exp(-coeff.decayRate * dt);
        accumulatedError += error * dt;


        float derivative = -cv;

        return (coeff.kP * error) + (coeff.kI * accumulatedError) + (coeff.kD * derivative) + (coeff.kF * tx);
    }

    public AnglePositionPID(PIDF.Weights _coeff) {
        accumulatedError = 0f;
        coeff = _coeff;
    }
}
