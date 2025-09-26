package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.RobotController;

public interface IAction <DataType>{

    void init(RobotController robot, DataType data);
    void start(RobotController robot, DataType data);
    void loop(RobotController robot, DataType data);
}
