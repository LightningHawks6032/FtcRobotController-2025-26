package org.firstinspires.ftc.teamcode.control;

public interface IControlLoop {
    float loop(float cx, float tx, float dt);

    default void reset() {}
}
