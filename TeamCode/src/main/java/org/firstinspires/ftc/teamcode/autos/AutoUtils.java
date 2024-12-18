package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoUtils {
    public static Command driveForwardWhileScoring(RobotContainer robotContainer) {
        return robotContainer.driveSubsystem.drive(
                () -> new ChassisSpeeds(0.3, 0, 0),
                () -> false).withTimeout(400);
    }

    public static Command driveToBasketAndScoreSample(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command moveToScoringSample1 = robotContainer.driveSubsystem.driveToPose(
                () -> new Pose2d(0.35, 0.78, Rotation2d.fromDegrees(135)),
                new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(5)),
                1);
        Command prepareToScore = robotContainer.superStructCommandsFactory.passSampleToUpperArm()
                .andThen(robotContainer.superStructureSubsystem
                        .moveToPose(SuperStructurePose.SCORE_SAMPLE.withArmFlipPosition(0.5)));
        sequence.addCommands(moveToScoringSample1.alongWith(prepareToScore));
        sequence.addCommands(robotContainer.superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SAMPLE));
        sequence.addCommands(robotContainer.superStructureSubsystem.openArmClaw());

        return sequence;
    }
}
