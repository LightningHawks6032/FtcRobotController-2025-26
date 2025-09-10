package org.firstinspires.ftc.teamcode.control;

public interface IControlLoop <TInput, TTarget, TWeights>{
    public TTarget loop(TInput input);
    public void setTarget(TTarget target);
    public void setWeights(TWeights weights);
}
