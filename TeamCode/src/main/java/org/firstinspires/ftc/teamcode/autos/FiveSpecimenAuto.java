package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

final class Positions1 {
    public static final Translation2d SCORE_PRELOADED_SPECIMEN = new Translation2d(0.58, 0.2);
}

public class FiveSpecimenAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(robotContainer.driveSubsystem, robotContainer.superStructureSubsystem, robotContainer.odometry);

        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded specimen -->
        Command driveToScoreSpecimen1 = robotContainer.driveSubsystem.followPath(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(-20)),
                        new Translation2d[]{new Translation2d(0.5, 0.2)},
                        new Pose2d(Positions1.SCORE_PRELOADED_SPECIMEN, Rotation2d.kZero),
                        Rotation2d.kZero,
                        0.5)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SPECIMEN))
                .andThen(AutoUtils.driveForwardWhileScoring(robotContainer));
        sequence.addCommands(driveToScoreSpecimen1);
        Command scoreSpecimen1 = AutoUtils.driveForwardWhileScoring(robotContainer)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(
                        SuperStructurePose.SCORE_SPECIMEN.withElevatorPosition(0.5)));
        sequence.addCommands(scoreSpecimen1);
        
        // Step2: Push first colored sample to source zone
        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                new Pose2d(Positions1.SCORE_PRELOADED_SPECIMEN, Rotation2d.k180deg),
                new Translation2d[] {new Translation2d(0.68, -0.3), new Translation2d(1.33, -0.3)},
                new Pose2d(1.45, -0.68, Rotation2d.kCW_90deg),
                Rotation2d.kZero,
                0.5)
                .alongWith(robotContainer.superStructCommandsFactory.holdIntake()));

        sequence.addCommands(pushSpecimenSequence(robotContainer, -0.68));

//        sequence.addCommands(robotContainer.driveSubsystem.followPath(
//                new Pose2d(Positions1.SCORE_PRELOADED_SPECIMEN, Rotation2d.k180deg),
//                new Translation2d[] {new Translation2d()},
//                new Pose2d(1.45, -0.68, Rotation2d.kCW_90deg),
//                Rotation2d.kZero,
//                0.5));


        return sequence;
    }

    static Command pushSpecimenSequence(RobotContainer robotContainer, double specimenYPosition) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        sequence.addCommands(robotContainer.driveSubsystem.followStraightLine(
                        new Translation2d(1.45, specimenYPosition),
                        new Translation2d(1.37, specimenYPosition),
                        Rotation2d.kZero,
                        0.5)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE))
                .alongWith(robotContainer.superStructureSubsystem.openIntakeClaw()));
        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE));
        sequence.addCommands(robotContainer.superStructureSubsystem.closeIntakeClaw());
        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));

        sequence.addCommands(robotContainer.driveSubsystem.followStraightLine(
                        new Translation2d(1.37, specimenYPosition),
                        new Translation2d(0.62, specimenYPosition),
                        Rotation2d.kZero,
                        0.7)
                .alongWith(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE.withExtendPosition(1))));
        sequence.addCommands(robotContainer.superStructureSubsystem.openIntakeClaw());

        return sequence;
    }
}
