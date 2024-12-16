package org.firstinspires.ftc.teamcode.autos;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class ExampleAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        final double speedMultiplier = 0.5;

        final Pose2d[] wayPoints = new Pose2d[]{
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(45)),
                new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(90))};
        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                wayPoints,
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                FollowPathCommand.reversePath(wayPoints),
                Rotation2d.fromDegrees(0),
                speedMultiplier));

        sequence.addCommands(new WaitCommand(5000));

        return sequence;
    }
}
