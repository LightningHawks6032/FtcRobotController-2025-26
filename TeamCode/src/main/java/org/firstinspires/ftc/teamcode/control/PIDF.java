package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

public class PIDF {

    public static class Weights {
        final float kP;
        final float kI;
        final float kD;
        final float kF;
        final float decayRate;
        final float lowPassFilter;

        public Weights(float _kP, float _kI, float _kD, float _kF, float _decayRate, float _lowPassFilter) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            decayRate = _decayRate;
            lowPassFilter = _lowPassFilter;
        }
    }

    public static class Controller implements IControlLoop{
        Weights weights;
        float accumulatedError;
        float prev_error;
        float previousFilteredDerivative;

        public void setWeights(Weights _weights) {
            weights = _weights;
        }

        public float loop(float _current_x, float _target_x, float _dt) {
            float x_error = _target_x - _current_x;

            if (x_error <= 1e-3) {
                x_error = 0;
            }


            accumulatedError *= (float) Math.exp(-weights.decayRate * _dt);
            accumulatedError += x_error * _dt;

            float derivative = (x_error - prev_error) / _dt;

            float filteredDerivative = derivative * weights.lowPassFilter + previousFilteredDerivative * (1 - weights.lowPassFilter);
            previousFilteredDerivative = filteredDerivative;

            prev_error = x_error;

            return (weights.kP * x_error + weights.kI * accumulatedError + weights.kD * filteredDerivative + _target_x * weights.kF);
        }

        public Controller(Weights _weights) {
            weights = _weights;


            accumulatedError = 0;
            prev_error = 0;


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
