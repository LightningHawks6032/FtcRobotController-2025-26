package org.firstinspires.ftc.teamcode.opmodes.teleop.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A;

@TeleOp(group="Testing", name="Verify Opmode")
public class VerifyRobotOpmode extends OpMode {

    TeleOpmode<ClankerHawk2A> opmode;




    @Override
    public void init() {
        opmode = new TeleOpmode<>(
                this,
                new ClankerHawk2A(hardwareMap),
                (r, b) -> b


                        .build(),
                TeleOpmode.EmptyGamepad()
        );
    }

    @Override
    public void loop() {

    }
}
