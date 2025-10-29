package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

public class CompoundControlLoop {
    public static class Controller implements IControlLoop {

        IControlLoop controlHi, controlLo;
        float threshold;


        @Override
        public float loop(float cx, float tx, float dt) {
            float error = tx - cx;
            if (error > threshold) {
                return controlHi.loop(cx, tx, dt);
            }
            return controlLo.loop(cx, tx, dt);
        }

        public Controller(IControlLoop _hi, IControlLoop _lo, float _threshold) {
            controlHi = _hi;
            controlLo = _lo;
            threshold = _threshold;
        }

        public Controller(@NonNull IControlLoopBuildOpt<? extends IControlLoop> _hi, @NonNull IControlLoopBuildOpt<? extends IControlLoop> _lo, float _threshold) {
            controlHi = _hi.build();
            controlLo = _lo.build();
            threshold = _threshold;
        }
    }

    public static class BuildOpt implements IControlLoopBuildOpt<Controller> {
        IControlLoopBuildOpt<? extends IControlLoop> hi, lo;
        float thresh;

        public BuildOpt(IControlLoopBuildOpt<? extends IControlLoop> _hi, IControlLoopBuildOpt<? extends IControlLoop> _lo, float _threshold) {
            hi = _hi;
            lo = _lo;
            thresh = _threshold;
        }

        @Override
        public Controller build() {
            return new Controller(hi, lo, thresh);
        }
    }

}
