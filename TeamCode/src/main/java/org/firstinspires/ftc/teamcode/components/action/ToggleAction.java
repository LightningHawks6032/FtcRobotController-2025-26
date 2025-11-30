package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.util.Toggle;

public class ToggleAction {
    Toggle toggle;

    public IAction<Boolean> build(IAction<Boolean> _action) {
        return new MapAction<>(
                _action,
                (robot, b) -> {
                    toggle.loop(b);
                    return toggle.toggle();
                }
        );
    }

    public ToggleAction() {
        toggle = new Toggle();
    }
}
