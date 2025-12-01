package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Testing extends OpMode {
    private DcMotor left_intake_motor;
    private DcMotor right_intake_motor;

    public void init() {
        // Hardware Mapping
        left_intake_motor = hardwareMap.dcMotor.get("left_intake_motor");
        right_intake_motor = hardwareMap.dcMotor.get("right_intake_motor");
        // Direction
        left_intake_motor.setDirection(DcMotor.Direction.REVERSE);
        right_intake_motor.setDirection(DcMotor.Direction.FORWARD);
        // Setting the encoding
        left_intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Braking is optional
        left_intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop () {
        // Controls
        double ishowspeed = (gamepad1.right_trigger);
        double liam = (gamepad1.left_trigger);
        double jkobe = (-liam);
        double speed = jkobe + ishowspeed;
        left_intake_motor.setPower(speed);
        right_intake_motor.setPower(speed);
        telemetry.addData("Motor power", speed);
    }
}
