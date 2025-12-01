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
        double ishowspeed = (gamepad1.right_trigger);
        double liam = (gamepad1.left_trigger);
        double jkobe = (-liam);
        double speed = jkobe + ishowspeed;
        front_left.setPower(speed);
        telemetry.addData("Motor power", speed);
    }
}
