package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

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


    public static class Controller implements IControlLoop{
        Weights weights;
        float accumulatedError;
        float prev_tx;

        //float previousFilteredDerivative;

        public void setWeights(Weights _weights) {
            weights = _weights;
        }

        public float loop(float _current_x, float _target_x, float _dt) {
            float x_error = _target_x - _current_x;
            // TODO: add deadband

            accumulatedError *= (float) Math.pow(weights.integralDecay, _dt);
            accumulatedError += x_error * _dt;

            float derivative = (_target_x - prev_tx) / _dt;

//        float filteredDerivative = derivative * weights.lowPassFilter + previousFilteredDerivative * (1 - weights.lowPassFilter);
//        previousFilteredDerivative = filteredDerivative;

            return (weights.kP * x_error + weights.kI * accumulatedError + weights.kD * derivative + _target_x * weights.kF);
        }

        public Controller(Weights _weights) {
            weights = _weights;


            accumulatedError = 0;
            prev_tx = 0;


            //previousFilteredDerivative = 0f;
        }
    }

    public static class BuildOpt implements IControlLoopBuildOpt<Controller> {

        Weights weights;

        public BuildOpt(@NonNull Weights _weights) {weights = _weights;}

        @Override
        public Controller build() {
            return new Controller(weights);
        }
    }
}
