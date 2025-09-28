package org.firstinspires.ftc.teamcode.hardware.drive;

public class PathPoint <DataType> {
    public DataType pos, vel, acc;

    public PathPoint(DataType _pos, DataType _vel, DataType _acc) {
        pos = _pos;
        vel = _vel;
        acc = _acc;
    }
}
