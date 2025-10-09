package org.firstinspires.ftc.teamcode.auto.action;


import java.util.function.Function;

public interface IDataProvider <ParamType, DataType> {
    Function<ParamType, DataType> getDataProvider();
}
