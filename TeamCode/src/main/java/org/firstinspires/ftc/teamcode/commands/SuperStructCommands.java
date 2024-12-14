package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructureSubsystem;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SuperStructCommands {

    private final SuperStructureSubsystem superStructureSubsystem;
    public SuperStructCommands(SuperStructureSubsystem superStructureSubsystem) {
        this.superStructureSubsystem = superStructureSubsystem;
    }

    public Command intakeContinuously(DoubleSupplier extendSpeed, DoubleSupplier intakeRotation, BooleanSupplier pushDownClawButton, BooleanSupplier closeClawButton) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(new InstantCommand(() -> superStructureSubsystem.setIntakeRotate(0)));
        sequence.addCommands(superStructureSubsystem.openArmClaw().alongWith(superStructureSubsystem.openIntakeClaw()));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));

        AtomicReference<Double> extendPosition = new AtomicReference<>(0.5);
        Supplier<SuperStructurePose> intakePose = () ->
                (pushDownClawButton.getAsBoolean() ? SuperStructurePose.INTAKE : SuperStructurePose.PREPARE_TO_INTAKE)
                        .withExtendPosition(extendPosition.get());
        final Command updateExtendPose = new RunCommand(() -> extendPosition.set(extendPosition.get() + 0.02 * extendSpeed.getAsDouble()));
        final Command updateRotation = new RunCommand(() -> superStructureSubsystem.setIntakeRotate(intakeRotation.getAsDouble()));
        final Command closeClawWhenRequested = new RunCommand(() -> superStructureSubsystem.setIntakeClaw(closeClawButton.getAsBoolean()));
        sequence.addCommands(
                superStructureSubsystem.follow(intakePose)
                        .alongWith(updateExtendPose)
                        .alongWith(updateRotation)
                        .alongWith(closeClawWhenRequested));
        return sequence;
    }

    public Command holdIntake() {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(superStructureSubsystem.closeIntakeClaw());
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.HOLD)
                .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(0)));
        return sequence;
    }

    public Command passToUpperStructure() {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(superStructureSubsystem.closeIntakeClaw());

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_PASS)
                .alongWith(superStructureSubsystem.openArmClaw())
                .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(0)));

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PASS));

        sequence.addCommands(superStructureSubsystem.closeArmClaw());
        sequence.addCommands(superStructureSubsystem.openIntakeClaw());

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.HOLD));

        return sequence;
    }
//
//    public Command scoreSample() {
//
//    }
//
//    public Command scoreSpecimen() {
//
//    }
}
