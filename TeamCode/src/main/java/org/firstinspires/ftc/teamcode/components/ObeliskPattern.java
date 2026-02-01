package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

public class ObeliskPattern <BallType> {
    final public BallType first, second, third;

    public ObeliskPattern(BallType _first, BallType _second, BallType _third) {
        first = _first;
        second = _second;
        third = _third;
    }

    @NonNull
    @Override
    public String toString() {
        return first.toString() + ", " + second.toString() + ", " + third.toString();
    }
}
