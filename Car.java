package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Car extends OpMode{
    private DcMotor front_left;
    public void init () {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
    }

    public void loop () {
        double right_trigger_value = (gamepad1.right_trigger);
        double left_trigger_value = (-gamepad1.left_trigger);
        double speed = left_trigger_value + right_trigger_value;
        front_left.setPower(speed);
        telemetry.addData("Motor power", speed);
    }
}
