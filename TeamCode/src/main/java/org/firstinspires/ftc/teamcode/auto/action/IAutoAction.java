package org.firstinspires.ftc.teamcode.auto.action;

import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.IAction;

///
/// `init` gets called on robot init <br>
/// `start` is called when the proceeding action terminates <br>
/// `loop` is called every frame
public interface IAutoAction <DataType extends ElapsedContainer> extends
        IAction<DataType>,
        ICompletionProvider,
        IDataProvider<Pair<Float, IRobot>, DataType> {
// TODO: Maybe provide init, start functions?
}
