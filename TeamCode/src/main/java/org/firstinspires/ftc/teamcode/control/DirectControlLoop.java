package org.firstinspires.ftc.teamcode.control;

public class DirectControlLoop{
    public static class Controller implements IControlLoop{
        @Override
        public float loop(float cx, float tx, float dt) {
            return tx;
        }

    }

    public static class BuildOpt implements IControlLoopBuildOpt<Controller> {
        @Override
        public Controller build() {
            return new Controller();
        }
    }
}
