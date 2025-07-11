package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.FiveSpecimenAuto.pushSpecimenSequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;



public class OneSpecimenPush implements Auto {

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
                        new Translation2d[] {new Translation2d(0.68, -0.45), new Translation2d(1.33, -0.50)},
                        new Pose2d(0.25, -0.85, Rotation2d.kCW_90deg),
                        Rotation2d.kZero,
                        0.5)
                .alongWith(robotContainer.superStructCommandsFactory.holdIntake()));



        return sequence;
    }
}
