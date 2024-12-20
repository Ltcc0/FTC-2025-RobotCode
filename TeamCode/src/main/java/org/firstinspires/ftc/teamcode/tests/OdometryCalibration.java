package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_ENCODER_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_RADIUS_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.utils.MapleLoopClock;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

// @TeleOp(name="Odometry Calibration")
public class OdometryCalibration extends OpMode {
    private final MapleLoopClock clock = new MapleLoopClock(50.0);
    private MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double radiansRotated, previousRotationRadians;
    private IMU imu;
    @Override
    public void init() {
        this.leftOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                LEFT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        this.rightOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                RIGHT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        this.centerOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                CENTER_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        this.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        this.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        this.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        this.backRight = hardwareMap.get(DcMotor.class, "backRight");

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(DriveTrainConstants.IMU_PARAMS);

        radiansRotated = 0;
        imu.resetYaw();
        previousRotationRadians = getIMUReading();

    }

    private double getIMUReading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();

        telemetry.addLine("rotate the robot with button A for 10 revolutions on field (3600 degrees) and press button x");
        telemetry.addData("left dis", leftOdometerWheel.getDistanceMeters());
        telemetry.addData("right dis", rightOdometerWheel.getDistanceMeters());
        telemetry.addData("center dis", centerOdometerWheel.getDistanceMeters());

        final double rotaryPower = gamepad1.a ? 0.3: -0;
        frontLeft.setPower(-rotaryPower * FRONT_LEFT_MOTOR_DIRECTION);
        backLeft.setPower(-rotaryPower * BACK_LEFT_MOTOR_DIRECTION);
        frontRight.setPower(rotaryPower * FRONT_RIGHT_MOTOR_DIRECTION);
        backRight.setPower(rotaryPower * BACK_RIGHT_MOTOR_DIRECTION);

        final double imuReading = getIMUReading();
        radiansRotated += Rotation2d.fromRadians(imuReading).minus(Rotation2d.fromRadians(previousRotationRadians)).getRadians();
        previousRotationRadians = imuReading;

        if (radiansRotated != 0) {
            final double trackWidth = (rightOdometerWheel.getDistanceMeters() - leftOdometerWheel.getDistanceMeters()) / radiansRotated,
                    centerWheelOffset = centerOdometerWheel.getDistanceMeters() / radiansRotated;
            telemetry.addData("track width (meters)", trackWidth); // 0.257
            telemetry.addData("center wheel offset (meters)", centerWheelOffset); // -0.022
        }

        telemetry.addData("rotation", Math.toDegrees(radiansRotated));

        telemetry.update();
        clock.tick();
    }
}
