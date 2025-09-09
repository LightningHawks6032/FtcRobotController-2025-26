package org.firstinspires.ftc.teamcode.control;

public interface IControlLoop <TInput, TTarget>{
    public TTarget loop(TInput input);
    public void setTarget(TTarget target);
}
