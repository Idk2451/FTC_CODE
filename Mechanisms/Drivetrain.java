package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Drivetrain {
    private IMU imu;

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private Servo arm;

    public double P_DRIVE_GAIN = 0.02;
    public double P_TURN_GAIN = 0.035;
    private double turnSpeed;

    public void init(@NonNull HardwareMap hardwareMap) {
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        arm = hardwareMap.get(Servo.class, "arm");

        //IMU HARDWARE MAPPING
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Setting directions
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Braking
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setPowers(double front_left_power, double back_left_power, double front_right_power , double back_right_power) {
        double maxSpeed  = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(front_left_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(back_left_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(front_right_power));
        maxSpeed = Math.max(maxSpeed, Math.abs(back_right_power));

        front_left_power /= maxSpeed;
        back_left_power /= maxSpeed;
        back_right_power /= maxSpeed;
        front_right_power /= maxSpeed;

        front_left.setPower(front_left_power);
        back_left.setPower(back_left_power);
        front_right.setPower(front_right_power);
        back_right.setPower(back_right_power);

    }
    public void drive (double forward, double right, double rotate) {
        double front_left_power = forward + right + rotate;
        double back_left_power = forward - right + rotate;
        double front_right_power = forward - right - rotate;
        double back_right_power = forward + right - rotate;
        setPowers(front_left_power, back_left_power, front_right_power, back_right_power);
    }

    public void runArm (double arm_position) {
        arm.setPosition(arm_position);
    }

    public double getArm() { return arm.getPosition(); }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = getHeading() - desiredHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return
                orientation.getYaw(AngleUnit.DEGREES);
    }
    public void turnToHeading(double rotate, double heading) {
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
            drive(0,0, turnSpeed);
    }
    public void driveStraight(double forward, double heading) {
        turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        drive(forward, 0, turnSpeed);
    }
    public void driveStrafe(double right, double heading) {
        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
        drive(0, right, turnSpeed);
    }
    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }
    public void resetYaw() {
        imu.resetYaw();
    }
}
