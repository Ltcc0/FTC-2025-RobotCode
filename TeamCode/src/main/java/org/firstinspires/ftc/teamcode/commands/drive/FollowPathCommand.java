package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.DriveControlLoops;
import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleTimer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import static org.firstinspires.ftc.teamcode.constants.DriveControlLoops.*;

public class FollowPathCommand extends CommandBase {
    private final Trajectory trajectory;
    private final double speedMultiplier;
    private final Rotation2d desiredRotation;
    private final double delaySecondsStartRotating;
    private final Pose2d tolerance;
    private final HolonomicDriveSubsystem driveSubsystem;

    private final MapleTimer timer;

    private double timeOutSeconds;

    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation) {
        this(trajectory, 1, driveSubsystem, desiredRotation);
    }

    public FollowPathCommand(Trajectory trajectory, double speedMultiplier, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation) {
        this(trajectory, speedMultiplier, driveSubsystem, desiredRotation, 0);
    }

    public FollowPathCommand(Trajectory trajectory, double speedMultiplier, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating) {
        this(trajectory, speedMultiplier, driveSubsystem, desiredRotation, pathProgressPercentageStartRotating, DriveControlLoops.tolerance);
    }

    public FollowPathCommand(Trajectory trajectory, double speedMultiplier, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating, Pose2d tolerance) {
        assert 0 < speedMultiplier && speedMultiplier <= 1 : "speed multiplier must be in the range 0~1";

        this.trajectory = trajectory;
        this.speedMultiplier = speedMultiplier;
        this.desiredRotation = desiredRotation;

        this.delaySecondsStartRotating = pathProgressPercentageStartRotating * trajectory.getTotalTimeSeconds();
        this.driveSubsystem = driveSubsystem;

        this.tolerance = tolerance;
        this.timer = new MapleTimer();

        super.addRequirements(driveSubsystem);

        withTimeOutAfterTrajectoryFinished(1);
    }

    @Override
    public void initialize() {
        driveController.setTolerance(tolerance);
        timer.reset();
        driveController.getThetaController().reset(driveSubsystem.getFacing().getRadians());
    }

    @Override
    public void execute() {
        final Trajectory.State state = scaleTime(trajectory.sample(getPathTime()), speedMultiplier);
        driveSubsystem.runRobotCentricChassisSpeeds(driveController.calculate(
                driveSubsystem.getPoseWithVelocityCompensation(
                        DriveControlLoops.TRANSLATIONAL_LOOK_AHEAD_TIME,
                        DriveControlLoops.ROTATIONAL_LOOK_AHEAD_TIME),
                state,
                getPathTime() > delaySecondsStartRotating ?
                        desiredRotation
                        : driveSubsystem.getFacing()
                ));
    }

    @Override
    public boolean isFinished() {
        if (timer.getTimeSeconds() > timeOutSeconds)
            return true;
        return getPathTime() >= trajectory.getTotalTimeSeconds()
                && driveController.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public double getPathTime() {
        return timer.getTimeSeconds() * speedMultiplier;
    }

    public FollowPathCommand withTimeOutAfterTrajectoryFinished(double timeOutAfterFinishSeconds) {
        this.timeOutSeconds = trajectory.getTotalTimeSeconds() * speedMultiplier + timeOutAfterFinishSeconds;
        return this;
    }

    public Command driveToStartingPointSequence(Pose2d startingPointTolerance, double navigateTimeOutSeconds) {
        return driveSubsystem
                .driveToPose(
                        () -> new Pose2d(trajectory.getInitialPose().getTranslation(), driveSubsystem.getFacing()),
                        startingPointTolerance,
                        navigateTimeOutSeconds)
                .andThen(this);
    }

    private static Trajectory.State scaleTime(Trajectory.State rawState, double speedMultiplier) {
        return new Trajectory.State(
                rawState.timeSeconds,
                rawState.velocityMetersPerSecond * speedMultiplier,
                rawState.accelerationMetersPerSecondSq * speedMultiplier * speedMultiplier,
                rawState.poseMeters,
                rawState.curvatureRadPerMeter);
    }

    public static Pose2d[] reversePath(Pose2d[] pathWayPoints) {
        final Pose2d[] revered = new Pose2d[pathWayPoints.length];
        for (int i = 0; i < pathWayPoints.length; i++) {
            final Pose2d originalWayPoint = pathWayPoints[pathWayPoints.length -1-i];
            revered[i] = new Pose2d(
                    originalWayPoint.getTranslation(),
                    originalWayPoint.getRotation().plus(Rotation2d.k180deg));
        }

        return revered;
    }
}
