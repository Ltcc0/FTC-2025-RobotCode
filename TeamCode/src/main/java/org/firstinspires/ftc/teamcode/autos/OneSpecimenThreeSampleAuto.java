package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class OneSpecimenThreeSampleAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(robotContainer.driveSubsystem, robotContainer.superStructureSubsystem, robotContainer.odometry);

        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded specimen -->
        Command driveToScoreSpecimen1 = robotContainer.driveSubsystem.followPath(
                new Pose2d(0, 0, Rotation2d.fromDegrees(-20)),
                new Translation2d[]{new Translation2d(0.5, -0.25)},
                new Pose2d(0.62, -0.25, Rotation2d.kZero),
                Rotation2d.kZero,
                0.5)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SPECIMEN))
                .andThen(AutoUtils.driveForwardWhileScoring(robotContainer));
        sequence.addCommands(driveToScoreSpecimen1);
        Command scoreSpecimen1 = AutoUtils.driveForwardWhileScoring(robotContainer)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(
                        SuperStructurePose.SCORE_SPECIMEN.withElevatorPosition(0.5)));
        sequence.addCommands(scoreSpecimen1);

        // <-- Step2: Score Sample -->
        Command moveToFirstSample = robotContainer.driveSubsystem.followPath(
                new Pose2d(0.62, -0.25, Rotation2d.k180deg),
                new Translation2d[]{new Translation2d(0.3, 0.5)},
                new Pose2d(0.6, 0.8, Rotation2d.kCCW_90deg),
                Rotation2d.k180deg,
                0.5);
        Command prepareToGrab = robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE)
                .andThen(robotContainer.superStructureSubsystem.openIntakeClaw());
        sequence.addCommands(moveToFirstSample.alongWith(prepareToGrab));

        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE));
        sequence.addCommands(robotContainer.superStructureSubsystem.closeIntakeClaw());

        sequence.addCommands(AutoUtils.driveToBasketAndScoreSample(robotContainer));

        // <-- Step3: Score Sample -->
        Command retrieveArm1 = new WaitCommand(500)
                .andThen(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));
        sequence.addCommands(robotContainer.driveSubsystem.driveToPose(
                () -> new Pose2d(0.6 , 0.71, Rotation2d.k180deg),
                new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(5)),
                1)
                .alongWith(retrieveArm1)
                .alongWith(robotContainer.superStructureSubsystem.openIntakeClaw()));

        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE));
        sequence.addCommands(robotContainer.superStructureSubsystem.closeIntakeClaw());

        sequence.addCommands(AutoUtils.driveToBasketAndScoreSample(robotContainer));

        // Step 4 Score Sample
        Command retrieveArm2 = new WaitCommand(500)
                .andThen(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));
        sequence.addCommands(robotContainer.driveSubsystem.driveToPose(
                        () -> new Pose2d(0.6, 0.74, Rotation2d.fromDegrees(-150)),
                        new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(5)),
                        1)
                .alongWith(retrieveArm2)
                .alongWith(robotContainer.superStructureSubsystem.openIntakeClaw())
                .beforeStarting(() -> robotContainer.superStructureSubsystem.setIntakeRotate(0.3)));

        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE));
        sequence.addCommands(robotContainer.superStructureSubsystem.closeIntakeClaw());

        sequence.addCommands(AutoUtils.driveToBasketAndScoreSample(robotContainer));

        // Step 5: park
        Command retrieveArm3 = new WaitCommand(500)
                .andThen(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));
        return sequence;
    }
}
