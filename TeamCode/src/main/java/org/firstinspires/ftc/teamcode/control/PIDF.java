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

    public static class Options {
        protected boolean rateLimited;
        protected float rateLimit_seconds;

        protected boolean deadbanded;
        protected float deadband;

        protected boolean clamp_integrators;
        protected float integrator_clamp;

        protected boolean bounded;
        protected Vec2 bounds;

        public Options() {
            rateLimited = false;
            deadbanded = false;
            clamp_integrators = false;
            bounded = false;
            rateLimit_seconds = 0;
            deadband = 0;
            integrator_clamp = 0;
            bounds = new Vec2(0, 0);
        }

        public Options ratelimit(float _ratelimit_seconds) {
            rateLimited = true;
            rateLimit_seconds = _ratelimit_seconds;
            return this;
        }

        public Options deadband(float _deadband) {
            deadbanded = true;
            deadband = _deadband;
            return this;
        }

        public Options integrator_clamp(float _clamp) {
            integrator_clamp = _clamp;
            clamp_integrators = true;
            return this;
        }
    
        public Options bound(Vec2 _bound) {
            bounds = _bound;
            bounded = true;
            return this;
        }
    }

    Weights weights;
    Options options;

    float accumulatedError;
    float previousError;
    ElapsedTime timer;

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

    public Options options() {return this.options;}

    @Override
    public Float loop(Float current) {
        float dt = (float) timer.seconds();

        if (options.rateLimited && dt < options.rateLimit_seconds) {
               return previousOutput;
        }
        timer.reset();

        float error = target - current;

        if (options.deadbanded && Math.abs(error) <= options.deadband) {
            return 0f;
        }

        accumulatedError *= (float) Math.pow(weights.integralDecay, dt);
        accumulatedError += error * dt;

        if (options.clamp_integrators) {
            accumulatedError = Util.clamp(accumulatedError, -options.integrator_clamp, options.integrator_clamp);
        }

        if (dt <= 1e-6) {dt = 1e-6f;}

        float derivative = (error - previousError) / dt;
        previousError = error;

        float filteredDerivative = derivative * weights.lowPassFilter + previousFilteredDerivative * (1 - weights.lowPassFilter);
        previousFilteredDerivative = filteredDerivative;

        float ret = (weights.kP * error + weights.kI * accumulatedError + weights.kD * filteredDerivative + target * weights.kF);

        if (options.bounded) {
            ret = Util.clamp(ret, options.bounds.x, options.bounds.y);
        }

        if (options.rateLimited) {
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

    public PIDF(Weights _weights) {
        weights = _weights;


        accumulatedError = 0;
        previousError = 0;
        target = 0;

        previousFilteredDerivative = 0f;

        timer = new ElapsedTime();
        previousOutput = 0;
    }


}
