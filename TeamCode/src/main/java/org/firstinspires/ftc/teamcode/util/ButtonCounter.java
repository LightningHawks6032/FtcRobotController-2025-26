package org.firstinspires.ftc.teamcode.util;

public class ButtonCounter {
    boolean toggle, hasLetGo;
    int count = 0;
    //public boolean toggle() {return toggle;}
    public int count() {return count;}
    //public void setToggle(boolean next) {toggle = next;}

    public ButtonCounter() {
        toggle = false;
        hasLetGo = true;
    }
    public ButtonCounter(boolean _toggle) {
        this();
        toggle = _toggle;
    }

    public void loop(boolean buttonData) {
        if (!buttonData) {
            hasLetGo = true;
        }
        else if (hasLetGo) {
            toggle = !toggle;
            count++;
            hasLetGo = false;
        }
    }
}
