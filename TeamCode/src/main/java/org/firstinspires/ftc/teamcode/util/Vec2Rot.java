package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class Vec2Rot {
    public float x;
    public float y;
    public float r;
    public Vec2Rot(float _x, float _y, float _r) {
        x = _x;
        y = _y;
        r = _r;
    }
    public Vec2Rot(@NonNull Vec2 p, float _r) {
        x = p.x;
        y = p.y;
        r = _r;
    }

    public Vec2 asVec2() {
        return new Vec2(x, y);
    }
}
