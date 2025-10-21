package org.firstinspires.ftc.teamcode.control;

public interface IControlLoopBuildOpt <ControlLoopType extends IControlLoop> {
    ControlLoopType build();
}
