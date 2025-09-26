package org.firstinspires.ftc.teamcode.control;

public class PIDF {

    public static class Weights {
        final float kP;
        final float kI;
        final float kD;
        final float kF;
        final float integralDecay;
        final float lowPassFilter;

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

    float previousFilteredDerivative;

    public void setWeights(Weights _weights) {
        weights = _weights;
    }

    public Float loop(float _current_x, float _target_x, float _target_v, float _current_v, float _dt) {
        float x_error = _target_x - _current_x;
        float v_error = _target_v - _current_v;

        // TODO: add deadband

        accumulatedError *= (float) Math.pow(weights.integralDecay, _dt);
        accumulatedError += x_error * _dt;

        float derivative = _target_v - _current_v;

//        float filteredDerivative = derivative * weights.lowPassFilter + previousFilteredDerivative * (1 - weights.lowPassFilter);
//        previousFilteredDerivative = filteredDerivative;

        float ret = (weights.kP * x_error + weights.kI * accumulatedError + weights.kD * accumulatedError + _target_x * weights.kF);

        return ret;
    }

    public PIDF(Weights _weights) {
        weights = _weights;


        accumulatedError = 0;


        //previousFilteredDerivative = 0f;
    }


}
