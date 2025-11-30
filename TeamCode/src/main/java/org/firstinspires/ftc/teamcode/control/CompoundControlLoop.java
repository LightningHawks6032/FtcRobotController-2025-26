package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

public class CompoundControlLoop {
    public static class Controller implements IControlLoop {

        private final IControlLoop farControl;
        private final IControlLoop nearControl;

        private final float enterFar;
        private final float exitFar;

        private boolean usingFar = true;

        @Override
        public float loop(float cx, float tx, float dt) {
            float error = Math.abs(tx - cx);

            if (usingFar) {
                if (error < exitFar)
                    usingFar = false;
            } else {
                if (error > enterFar)
                    usingFar = true;
            }

            return usingFar
                    ? farControl.loop(cx, tx, dt)
                    : nearControl.loop(cx, tx, dt);
        }

        public Controller(IControlLoop _far, IControlLoop _near, float _enterFar, float _exitFar) {

            assert (_exitFar < _enterFar);

            farControl = _far;
            nearControl = _near;
            enterFar = _enterFar;
            exitFar = _exitFar;
        }

        public Controller(@NonNull IControlLoopBuildOpt<? extends IControlLoop> _far, @NonNull IControlLoopBuildOpt<? extends IControlLoop> _near, float _enterFar, float _exitFar) {
            this(_far.build(), _near.build(), _enterFar, _exitFar);
        }
    }

    public static class BuildOpt implements IControlLoopBuildOpt<Controller> {
        IControlLoopBuildOpt<? extends IControlLoop> hi, lo;
        float enterFar, exitFar;

        public BuildOpt(IControlLoopBuildOpt<? extends IControlLoop> _hi, IControlLoopBuildOpt<? extends IControlLoop> _lo, float _enterFar, float _exitFar) {
            hi = _hi;
            lo = _lo;
            enterFar = _enterFar;
            exitFar = _exitFar;
        }

        @Override
        public Controller build() {
            return new Controller(hi, lo, enterFar, exitFar);
        }
    }

}

