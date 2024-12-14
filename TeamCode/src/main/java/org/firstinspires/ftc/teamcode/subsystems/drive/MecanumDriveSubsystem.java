package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveControlLoops;
import org.firstinspires.ftc.teamcode.constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDriveSubsystem {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final MecanumDriveKinematics mecanumDriveKinematics;
    private final MapleOdometerWheelsOdometry odometry;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, MapleOdometerWheelsOdometry odometry) {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR_NAME);
        this.odometry = odometry;

        this.mecanumDriveKinematics = DriveTrainConstants.KINEMATICS;
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond * DriveControlLoops.HORIZONTAL_MOVEMENT_COMPENSATION_FACTOR,
                speeds.omegaRadiansPerSecond);

        final MecanumDriveWheelSpeeds wheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(speeds);
        runDriveOpenLoop(frontLeft, wheelSpeeds.frontLeftMetersPerSecond * FRONT_LEFT_MOTOR_DIRECTION);
        runDriveOpenLoop(frontRight, wheelSpeeds.frontRightMetersPerSecond * FRONT_RIGHT_MOTOR_DIRECTION);
        runDriveOpenLoop(backLeft, wheelSpeeds.rearLeftMetersPerSecond * BACK_LEFT_MOTOR_DIRECTION);
        runDriveOpenLoop(backRight, wheelSpeeds.rearRightMetersPerSecond * BACK_RIGHT_MOTOR_DIRECTION);

        SystemConstants.telemetry.addData("Chassis Speeds", speeds);
        SystemConstants.telemetry.addData("Wheel Speeds", wheelSpeeds);
    }

    private void runDriveOpenLoop(DcMotor motor, double desiredSpeedMetersPerSecond) {
        desiredSpeedMetersPerSecond = MathUtil.applyDeadband(
                desiredSpeedMetersPerSecond,
                DriveControlLoops.DRIVE_DEAD_BAND_MPS,
                MAX_VELOCITY_METERS_PER_SECOND);
        motor.setPower(DriveControlLoops.driveFeedForward.calculate(desiredSpeedMetersPerSecond));
    }

    @Override
    public Pose2d getPose() {
        return odometry.getEstimatedPose();
    }

    private Rotation2d getRobotAngle() {
        return odometry.getEstimatedPose().getRotation();
    }

    @Override
    public void setPose(Pose2d currentPose) {
        odometry.resetPose(currentPose);
    }

    @Override
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return odometry.getMeasureSpeedsRobotRelative();
    }

    @Override
    public double getChassisMaxLinearVelocity() {
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    @Override
    public double getChassisMaxAngularVelocity() {
        return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        odometry.addVisionMeasurement(visionPose, timestamp);
    }
}
