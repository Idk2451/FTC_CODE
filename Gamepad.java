package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class Gamepad extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addData("DPAD Right", gamepad1.dpad_right);
        telemetry.addData("DPAD Down", gamepad1.dpad_down);
        telemetry.addData("DPAD Left", gamepad1.dpad_left);
        telemetry.addData("DPAD Up", gamepad1.dpad_up);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("B", gamepad1.b);
        telemetry.addData("X", gamepad1.x);
        telemetry.addData("Y", gamepad1.y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Right Stick Button", gamepad1.right_stick_button);
    }
}
