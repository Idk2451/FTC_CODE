package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@TeleOp
public class Car extends OpMode {
    private DcMotor core_motor;
    Sensors sensor = new Sensors();
    private String  indicator;


    @Override
    public void init() {
        core_motor = hardwareMap.get(DcMotor.class, "front_right");
        sensor.init(hardwareMap);
        indicator = "front_right";
    }

    @Override
    public void loop() {
        telemetry.addData("This class is only for testing components and these tests are decided upon by the programmer", null);
        telemetry.addData("front_right is right on DPAD \n front_left is left on DPAD \n back_right is up on DPAD \n back_left is down on DPAD. \n Choose any of these to switch motor", null);
        float right_trigger_value = (gamepad1.right_trigger);
        float left_trigger_value = (gamepad1.left_trigger);
        float speed = left_trigger_value + right_trigger_value;
        core_motor.setPower(speed);
        sensor.tailLights(speed);
        telemetry.addData("Motor power", speed);
        telemetry.addData("Your are using motor: ", indicator);


        if (gamepad1.dpad_right) {
            core_motor = hardwareMap.get(DcMotor.class, "front_right");
            indicator = "front_right";
        }
        if (gamepad1.dpad_left) {
            core_motor = hardwareMap.get(DcMotor.class, "front_left");
            indicator = "front_left";
        }
        if (gamepad1.dpad_up) {
            core_motor = hardwareMap.get(DcMotor.class, "back_right");
            indicator = "back_right";
        }
        if (gamepad1.dpad_down) {
            core_motor = hardwareMap.get(DcMotor.class, "back_left");
            indicator = "back_left";
        }
        telemetry.update();
    }
}
