package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ResetIMU extends OpMode {
    Drivetrain drive = new Drivetrain();

    @Override
    public void init() {
        drive.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("Hello press the A button to reset the IMU", null);
        if (gamepad1.a) {
            drive.imu.resetYaw();
        }
    }
}
