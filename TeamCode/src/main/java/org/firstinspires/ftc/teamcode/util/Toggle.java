package org.firstinspires.ftc.teamcode.util;

public class Toggle {
    boolean toggle, hasLetGo;
    public boolean toggle() {return toggle;}


    public Toggle() {
        toggle = false;
        hasLetGo = true;
    }
    public Toggle(boolean _toggle) {
        this();
        toggle = _toggle;
    }

    public void loop(boolean buttonData) {
        if (!buttonData) {
            hasLetGo = true;
        }
        else if (hasLetGo) {
            toggle = !toggle;
            hasLetGo = false;
        }
    }
}
