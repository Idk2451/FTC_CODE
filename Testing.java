package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Testing extends OpMode {
    private DcMotor front_left;
    public void init() {
        // Hardware Mapping
        front_left = hardwareMap.dcMotor.get("launch_motor");
        // Direction
        front_left.setDirection(DcMotor.Direction.FORWARD);
        // Setting the encoding
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop () {
        // Controls
        float right_trigger_value = (gamepad1.right_trigger);
        float left_trigger_value = (-gamepad1.left_trigger);
        float speed = left_trigger_value + right_trigger_value;
        front_left.setPower(speed);
        telemetry.addData("Motor power", speed);
    }
}
