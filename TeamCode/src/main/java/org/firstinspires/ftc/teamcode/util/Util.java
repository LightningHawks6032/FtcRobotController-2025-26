package org.firstinspires.ftc.teamcode.util;

import java.util.function.Consumer;

public class Util {
    public static float clamp(float val, float min, float max) {
        return Math.max(Math.min(val, max), min);
    }

    public static <T> T also(T obj, Consumer<T> fun) {
        fun.accept(obj);
        return obj;
    }
}
