package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class ColorRGBA {
    public float r, g, b, a;

    public ColorRGBA(float _r, float _g, float _b, float _a) {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }

    @NonNull
    @Override
    public String toString() {
        return r + ", " + g + ", " + b + ", " + a;
    }

    public float distanceSquared(@NonNull ColorRGBA other) {
        float _r = r - other.r;
        float _g = g - other.g;
        float _b = b - other.b;

        return _r * _r + _g * _g + _b * _b;
    }
}

