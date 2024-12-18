package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * Stores the information of your chassis
 * TODO: you have to change this file to match your robot
 * */
public class DriveTrainConstants {
    public static final String
        FRONT_LEFT_MOTOR_NAME = "frontLeft",
        FRONT_RIGHT_MOTOR_NAME = "frontRight",
        BACK_LEFT_MOTOR_NAME = "backLeft",
        BACK_RIGHT_MOTOR_NAME = "backRight";

    public static final String
            CENTER_ODOMETER_WHEEL_NAME = "frontRight",
            LEFT_ODOMETER_WHEEL_NAME = "backLeft",
            RIGHT_ODOMETER_WHEEL_NAME = "backRight";

    /* if the RAW encoder reading is POSITIVE when moving to the left, false; other wise, true */
    public static final boolean CENTER_ODOMETER_WHEEL_INVERTED = true;
    /* if the RAW encoder reading is POSITIVE when moving front, false; other wise, true */
    public static final boolean LEFT_ODOMETER_WHEEL_INVERTED = true;
    public static final boolean RIGHT_ODOMETER_WHEEL_INVERTED = false;

    public static final double
            FRONT_LEFT_MOTOR_DIRECTION = 1,
            FRONT_RIGHT_MOTOR_DIRECTION = -1,
            BACK_LEFT_MOTOR_DIRECTION = 1,
            BACK_RIGHT_MOTOR_DIRECTION = -1;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));

    public static final double ODOMETER_ENCODER_TICKS_PER_REVOLUTION = 2048; // 2048 ticks mag encoder
    public static final double ODOMETER_WHEELS_RADIUS_METERS = 24.0/1000; // 24mm

    public static final double ODOMETER_WHEELS_TRACK_WIDTH_METERS = 0.273;
    public static final double ODOMETER_CENTER_WHEELS_OFFSET = -0.096;

    public static final double CHASSIS_WIDTH_METERS = 0.3;
    public static final double CHASSIS_LENGTH_METERS = 0.4;

    public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(
            new Translation2d(CHASSIS_LENGTH_METERS /2, CHASSIS_WIDTH_METERS / 2),
            new Translation2d(CHASSIS_LENGTH_METERS /2, -CHASSIS_WIDTH_METERS / 2),
            new Translation2d(-CHASSIS_LENGTH_METERS /2, CHASSIS_WIDTH_METERS / 2),
            new Translation2d(-CHASSIS_LENGTH_METERS /2, -CHASSIS_WIDTH_METERS / 2));

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.8, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(360);
}
