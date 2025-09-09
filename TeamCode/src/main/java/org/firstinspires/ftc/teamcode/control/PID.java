package org.firstinspires.ftc.teamcode.control;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;

public class PID implements IControlLoop<Float, Float> {
    float accumulatedError;
    float previousError;
    ElapsedTime timer;

    float proportionalWeight, integralWeight, derivativeWeight;

    boolean bounded;
    Vec2 bounds;

    boolean rateLimited;
    float rateLimit_seconds;
    float previousOutput;

    float target;



    @Override
    public void setTarget(Float _target) {
        target = _target;
    }
    @Override
    public Float loop(Float dx) {
        float dt = (float) timer.time();

        if (rateLimited && dt < rateLimit_seconds) {
               return previousOutput;
        }

        accumulatedError *= Math.pow(0.75, dt);
        timer.reset();
        Log.d("Dt", "" + dt);
        if (dt <= 1e-6) {dt = 1e-6f;}
        accumulatedError += dx * dt;
        // add error clamp/saturation?

        float v = dx / dt;

        float ret = (proportionalWeight * dx + integralWeight * accumulatedError + derivativeWeight * v);

        if (bounded) {
            ret = Util.clamp(ret, bounds.x, bounds.y);
        }

        if (rateLimited) {
            previousOutput = ret;
        }

        return ret;
    }

    public PID addBounds (float min, float max) {
        bounds = new Vec2(min, max);
        bounded = true;
        return this;
    }

    public PID setRateLimit(float _rateLimit_seconds) {
        rateLimited = true;
        rateLimit_seconds = _rateLimit_seconds;
        return this;
    }

    public PID(float P, float I, float D) {
        proportionalWeight = P;
        integralWeight = I;
        derivativeWeight = D;

        accumulatedError = 0;
        previousError = 0;
        target = 0;

        bounded = false;
        rateLimited = false;

        timer = new ElapsedTime();
        previousOutput = 0;
    }
}
