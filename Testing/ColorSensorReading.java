package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Sensors;

@TeleOp
public class ColorSensorReading extends OpMode {
    Sensors sensor = new Sensors();

    @Override
    public void init() {
        sensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Color Sensor amount", sensor.getAmountLeftBlue());
        telemetry.addData("Right Color Sensor amount", sensor.getAmountRightBlue());
        telemetry.update();
    }
}
