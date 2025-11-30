package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

public class LazyInit <T> {
    T obj;
    final Supplier<T> init;

    public T get() {
        if (obj == null) {
            obj = init.get();
        }
        return obj;
    }

    public LazyInit(Supplier<T> _init) {
        init = _init;
    }

}
