package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;

public class PIDF implements IControlLoop<Float, Float, PIDF.Weights> {

    public static class Weights {
        float kP, kI, kD, kF;
        float integralDecay, lowPassFilter;

        public Weights(float _kP, float _kI, float _kD, float _kF, float _integralDecay, float _lowPassFilter) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            integralDecay = _integralDecay;
            lowPassFilter = _lowPassFilter;
        }
    }

    Weights weights;

    float accumulatedError;
    float previousError;
    ElapsedTime timer;

    boolean bounded;
    Vec2 bounds;

    boolean rateLimited;
    float rateLimit_seconds;
    float previousOutput;

    float previousFilteredDerivative;
    float target;



    @Override
    public void setTarget(Float _target) {
        target = _target;
    }

    @Override
    public void setWeights(Weights _weights) {
        weights = _weights;
    }

    @Override
    public Float loop(Float x) {
        float dt = (float) timer.seconds();

        if (rateLimited && dt < rateLimit_seconds) {
               return previousOutput;
        }
        timer.reset();

        float error = target - x;

        accumulatedError *= Math.pow(weights.integralDecay, dt);
        accumulatedError += error * dt;

        if (dt <= 1e-6) {dt = 1e-6f;}

        float derivative = (error - previousError) / dt;
        previousError = error;

        float filteredDerivative = derivative * weights.lowPassFilter + previousFilteredDerivative * (1 - weights.lowPassFilter);
        previousFilteredDerivative = filteredDerivative;

        float ret = (weights.kP * error + weights.kI * accumulatedError + weights.kD * filteredDerivative + target * weights.kF);

        if (bounded) {
            ret = Util.clamp(ret, bounds.x, bounds.y);
        }

        if (rateLimited) {
            previousOutput = ret;
        }

        return ret;
    }

    public void reset(float x) {
        target = x;
        accumulatedError = 0;
        previousError = 0;
        previousOutput = 0;
    }

    public PIDF addBounds (float min, float max) {
        bounds = new Vec2(min, max);
        bounded = true;
        return this;
    }

    public PIDF setRateLimit(float _rateLimit_seconds) {
        rateLimited = true;
        rateLimit_seconds = _rateLimit_seconds;
        return this;
    }

    public PIDF(Weights _weights) {
        weights = _weights;

        accumulatedError = 0;
        previousError = 0;
        target = 0;

        previousFilteredDerivative = 0f;

        bounded = false;
        rateLimited = false;

        timer = new ElapsedTime();
        previousOutput = 0;
    }
}
