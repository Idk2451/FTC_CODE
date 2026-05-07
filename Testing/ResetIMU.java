package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

@TeleOp
public class ResetIMU extends OpMode {
    Drivetrain drive = new Drivetrain();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Press the A button to reset the IMU", null);
        if (gamepad1.a) {
            drive.resetYaw();
        }
    }
}
