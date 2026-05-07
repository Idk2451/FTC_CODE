package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    private LED back_left_led;
    private LED back_right_led;

    private DistanceSensor front_distance;
    private DistanceSensor back_distance;
    private DistanceSensor right_distance;
    private DistanceSensor left_distance;

    private ColorSensor left_color;
    private ColorSensor right_color;

    public void init(@NonNull HardwareMap hardwareMap) {
        back_left_led = hardwareMap.led.get("back_left_led");
        back_right_led = hardwareMap.led.get("back_right_led");

        front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
        back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
        left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");

        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
    }

    public void tailLights(double forward) {
        if (forward < 0) {
            back_left_led.on();
            back_right_led.on();
        }
        else {
            back_left_led.off();
            back_right_led.off();
        }
    }

    public int getAmountLeftBlue() {
        return left_color.blue();
    }

    public int getAmountRightBlue() {
        return right_color.blue();
    }

    public double getRobotLeft () {
        if (0 < left_distance.getDistance(DistanceUnit.CM) ||
                left_distance.getDistance(DistanceUnit.CM) < 200) {
            return left_distance.getDistance(DistanceUnit.CM);
        } else {
            return 250;
        }
    }

    public double getRobotRight () {
        if (0 < right_distance.getDistance(DistanceUnit.CM) ||
                right_distance.getDistance(DistanceUnit.CM) < 200) {
            return right_distance.getDistance(DistanceUnit.CM);
        } else {
            return 250;
        }
    }

    public double getRobotFront () {
        if (0 < front_distance.getDistance(DistanceUnit.CM) ||
                front_distance.getDistance(DistanceUnit.CM) < 200) {
            return front_distance.getDistance(DistanceUnit.CM);
        }
        else {
            return 250;
        }
    }

    public double getRobotBack () {
        if (0 < back_distance.getDistance(DistanceUnit.CM) ||
                back_distance.getDistance(DistanceUnit.CM) < 200) {
            return back_distance.getDistance(DistanceUnit.CM);
        }
        else {
            return 250;
        }
    }
}
