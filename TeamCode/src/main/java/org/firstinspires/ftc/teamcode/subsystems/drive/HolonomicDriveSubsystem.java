package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveControlLoops.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.commands.drive.FollowPathCommand;
import org.firstinspires.ftc.teamcode.constants.DriveControlLoops;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public interface HolonomicDriveSubsystem extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     * */
    void runRawChassisSpeeds(ChassisSpeeds speeds);

    /**
     * Returns the current odometry Pose.
     */
    Pose2d getPose();

    default Pose2d getPoseWithVelocityCompensation() {
        return getPoseWithVelocityCompensation(ROTATIONAL_LOOK_AHEAD_TIME, TRANSLATIONAL_LOOK_AHEAD_TIME);
    }


    default Pose2d getPoseWithVelocityCompensation(double translationLookAheadTime, double rotationalLookAheadTime) {
        final ChassisSpeeds robotSpeedsFieldRelative = getMeasuredChassisSpeedsFieldRelative();
        final Transform2d lookAheadTransform = new Transform2d(
                robotSpeedsFieldRelative.vxMetersPerSecond * translationLookAheadTime,
                robotSpeedsFieldRelative.vyMetersPerSecond * translationLookAheadTime,
                new Rotation2d(robotSpeedsFieldRelative.omegaRadiansPerSecond * rotationalLookAheadTime));

        return getPose().plus(lookAheadTransform);
    }

    default Rotation2d getFacing() {return getPose().getRotation(); }

    /**
     * Resets the current odometry Pose to a given Pose
     */
    void setPose(Pose2d currentPose);

    /**
     * @return the measured(actual) velocities of the chassis, robot-relative
     * */
    ChassisSpeeds getMeasuredChassisSpeedsRobotRelative();

    default ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    double getChassisMaxLinearVelocity();
    double getChassisMaxAngularVelocity();

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    void addVisionMeasurement(Pose2d visionPose, double timestamp);

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     * */
    default void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().getRotation()
        ));
    }

    default void stop() {
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     * */
    default void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
        runRawChassisSpeeds(ChassisSpeeds.discretize(
                speeds,
                1.0/ SystemConstants.ROBOT_UPDATE_RATE_HZ
        ));
    }

    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.omegaRadiansPerSecond == 0 && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0;
    }

    default Command drive(Supplier<ChassisSpeeds> chassisSpeedsSupplier, BooleanSupplier fieldCentricSwitch) {
        return new RunCommand(
                () -> {
                    if (fieldCentricSwitch.getAsBoolean())
                        runFieldCentricChassisSpeeds(chassisSpeedsSupplier.get());
                   else
                       runRobotCentricChassisSpeeds(chassisSpeedsSupplier.get());
                },
                this
        );
    }

    default Command driveToPose(Supplier<Pose2d> target) {
        return drive(
                () -> driveController.calculate(
                        this.getPoseWithVelocityCompensation(),
                        target.get(),
                        0,
                        target.get().getRotation()),
                () -> false)
                .beforeStarting(() -> driveController.getThetaController().reset(getFacing().getRadians()));
    }

    default Command driveToPose(Supplier<Pose2d> target, Pose2d tolerance, double timeOutSeconds) {
        final Trigger isStationary = velocityBelow(0.1);
        return driveToPose(target)
                .beforeStarting(() -> driveController.setTolerance(tolerance))
                .beforeStarting(() -> driveController.calculate(getPose(), target.get(), 0, target.get().getRotation()))
                .raceWith(new WaitUntilCommand(
                        () -> driveController.atReference() && isStationary.get()))
                .withTimeout((long)(timeOutSeconds * 1000))
                .whenFinished(this::stop);
    }

    default boolean isVelocityBelow(double velocityPercent) {
        final ChassisSpeeds speeds = getMeasuredChassisSpeedsRobotRelative();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                < getChassisMaxLinearVelocity() * velocityPercent
                && Math.abs(speeds.omegaRadiansPerSecond)
                < getChassisMaxAngularVelocity() * velocityPercent;
    }

    default Trigger velocityBelow(double velocityPercent) {
        return new Trigger(() -> isVelocityBelow(velocityPercent));
    }


    default Command followTrajectory(Supplier<Trajectory.State> desiredState, Supplier<Rotation2d> desiredRotation) {
        return drive(
                () -> driveController.calculate(
                        this.getPoseWithVelocityCompensation(
                                DriveControlLoops.TRANSLATIONAL_LOOK_AHEAD_TIME,
                                DriveControlLoops.ROTATIONAL_LOOK_AHEAD_TIME),
                        desiredState.get(),
                        desiredRotation.get()),
                () -> false);
    }


    default Command followPath(Pose2d startingPoint, Translation2d[] interiorWayPoints, Pose2d endingPoint, Rotation2d desiredRotation, double speedMultiplier) {
        return new FollowPathCommand(
                TrajectoryGenerator.generateTrajectory(
                        startingPoint,
                        Arrays.asList(interiorWayPoints),
                        endingPoint,
                        trajectoryConfig),
                speedMultiplier,
                this,
                desiredRotation,
                0)
                .whenFinished(this::stop);
    }

    default Command followStraightLine(Translation2d startingPoint, Translation2d endingPoint, Rotation2d desiredRotation, double speedMultiplier) {
        Rotation2d direction = endingPoint.minus(startingPoint).getAngle();
        Translation2d midPoint = startingPoint.plus(
                endingPoint.minus(startingPoint).div(2));

        return followPath(
                new Pose2d(startingPoint, direction),
                new Translation2d[]{midPoint},
                new Pose2d(endingPoint, direction),
                desiredRotation,
                speedMultiplier);
    }
}
