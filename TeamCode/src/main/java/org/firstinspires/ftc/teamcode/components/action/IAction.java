package org.firstinspires.ftc.teamcode.components.action;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.components.IRobot;
import org.jetbrains.annotations.Contract;

import java.util.function.BiConsumer;

public interface IAction <DataType>{
    void init(IRobot robot, DataType data);
    void start(IRobot robot, DataType data);
    void loop(IRobot robot, DataType data);

    class From {
        @NonNull
        @Contract("_ -> new")
        public static <Ty> IAction<Ty> loop(BiConsumer<IRobot, Ty> loop) {
            return new IAction<Ty>() {
                @Override
                public void init(IRobot robot, Ty data) {

                }

                @Override
                public void start(IRobot robot, Ty data) {

                }

                @Override
                public void loop(IRobot robot, Ty data) {
                    loop.accept(robot, data);
                }
            };
        }
    }

}