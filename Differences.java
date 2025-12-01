package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Differences extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y < 0) {
            telemetry.addData("Left Stick", "is negetive");
        }
        else{
            telemetry.addData("Left Stick", "is positive");
        }
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
    }
}

