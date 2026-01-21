package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.util.LazyInit;

public class SpindexerController {
    enum BallState {
        GREEN,
        PURPLE,
        NONE
    }

    static class BallSlot {
        public float pos;
        public BallState state;

        public BallSlot(float _pos, BallState _state) {
            pos = _pos;
            state = _state;
        }
    }

    static class IndexerSlots {
        BallSlot slot1, slot2, slot3;

        public boolean hasState(BallState state) {
            return slot1.state == state ||
                    slot2.state == state ||
                    slot3.state == state;
        }
        public BallSlot getNext(BallState state) {
            if (slot1.state == state) {
                return slot1;
            }
            else if (slot2.state == state) {
                return slot2;
            }
            else if (slot3.state == state) {
                return slot3;
            }

            return null;
        }

        public IndexerSlots(BallSlot _slot1, BallSlot _slot2, BallSlot _slot3) {
            slot1 = _slot1;
            slot2 = _slot2;
            slot3 = _slot3;
        }
    }

    DcMotorWrapper motor;
    LazyInit<IAction<Object>> goGoTargetPositionAction;

    public SpindexerController() {
        slot1 = new BallSlot(0, BallState.NONE);
        slot2 = new BallSlot(2f/3f * (float)Math.PI, BallState.NONE);
        slot3 = new BallSlot(4f/3f * (float)Math.PI, BallState.NONE);
    }
}