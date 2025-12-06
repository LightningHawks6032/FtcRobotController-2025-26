package org.firstinspires.ftc.teamcode.util;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

public class Util {
    public static float clamp(float val, float min, float max) {
        return Math.max(Math.min(val, max), min);
    }

    public static <T> T also(T obj, Consumer<T> fun) {
        fun.accept(obj);
        return obj;
    }

    ///  Easier than casting :3c
    public static <_In1, _In2, _Out> BiFunction<_In1, _In2, _Out> LambdaAsFunction(BiFunction<_In1, _In2, _Out> f) {
        return f;
    }

    public static <_In, _Out> Function<_In, _Out> LambdaAsFunction(Function<_In, _Out> f) {
        return f;
    }

    /// Normalizes angle between [-pi/2, pi/2]
    public static float normAngle(float a) {
        while (a < -Math.PI / 2) { a += (float)(2 * Math.PI);}
        while (a > Math.PI / 2) {a -= (float)(2 * Math.PI);}
        return a;
    }

    public static float normAngle2Pi(float a) {
        while (a < 0) {a += (float)(2 * Math.PI);}
        while (a > 2 * Math.PI) {a -= (float)(2 * Math.PI);}
        return a;
    }

}
