package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public interface IMotor {
    enum Direction {
        FORWARD, REVERSE;
        public static DcMotor.Direction toMotorDir(@NonNull Direction dir) {
            switch (dir) {
                case FORWARD:
                    return DcMotor.Direction.FORWARD;
                case REVERSE:
                    return DcMotorSimple.Direction.REVERSE;
            }
            return DcMotorSimple.Direction.FORWARD;
        }
        public static Direction fromMotorDir(@NonNull DcMotor.Direction dir) {
            switch (dir) {
                case FORWARD:
                    return FORWARD;
                case REVERSE:
                    return REVERSE;
            }
            return FORWARD;
        }
    }

    void setPower(float power);
    float getPower();
    void setTorque(float torque, float currentVelocity);
    int getPosition();
    void setVelocity(float velocity);
    float getVelocity();
    void setDirection(Direction dir);
    Direction getDirection();


}
