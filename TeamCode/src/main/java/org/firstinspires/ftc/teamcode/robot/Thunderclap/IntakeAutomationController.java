package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Toggle;

public class IntakeAutomationController {


    Toggle intakeToggle;

    LazyInit<IAction<Boolean>> intakeMode;
    public IAction<Boolean> intakeModeAction() {return intakeMode.get();}


    ThunderclapRobot robot;
    IAction<Boolean> goToEmpty = IAction.From.loop((r, b)-> {if (b && !robot.kickerController.isRunning()) robot.spindexerCommander.goToBallState().loop(r, SpindexerController.BallState.NONE);});


    public IntakeAutomationController(ThunderclapRobot _robot) {
        robot = _robot;
        intakeToggle = new Toggle();

        intakeMode = new LazyInit<>(() -> IAction.From.loop((r, b) -> {
            intakeToggle.loop(b);
            //goToEmpty.loop(robot, true);
            if (intakeToggle.toggle()) {
                robot.intakeSwingController.assertSwing.get().loop(robot, true);
            }
            else {
                robot.intakeSwingController.assertSwing.get().loop(robot, false);
            }
        }));
    }

}
