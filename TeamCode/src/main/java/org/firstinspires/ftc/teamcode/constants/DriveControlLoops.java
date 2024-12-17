package org.firstinspires.ftc.teamcode.constants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveControlLoops {
    public static final double DRIVE_DEAD_BAND_MPS = 0.03;
    public static final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.1, 1.0 / DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    public static final ProfiledPIDController rotationController = new ProfiledPIDController(
            3.5, 0, 0,
            new TrapezoidProfile.Constraints(DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 720));
    public static final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(3, 0, 0),
            new PIDController(3, 0, 0),
            rotationController);

    public static final double TRANSLATIONAL_LOOK_AHEAD_TIME = 0.25, ROTATIONAL_LOOK_AHEAD_TIME = 0.2;
    public static final double HORIZONTAL_MOVEMENT_COMPENSATION_FACTOR = 1.5;

    public static final Pose2d DEFAULT_TOLERANCE = new Pose2d(0.04, 0.04, Rotation2d.fromDegrees(5));
    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
            5);
}
