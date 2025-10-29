package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerWrapper {
    float last;
    ElapsedTime timer;

    public TimerWrapper() {
        last = 0f;
        timer = new ElapsedTime();
    }

    public void reset() {
        last = (float)timer.seconds();
    }

    public float get() {
        return (float) timer.seconds() - last;
    }

}
