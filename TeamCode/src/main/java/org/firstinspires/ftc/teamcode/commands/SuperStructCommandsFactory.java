package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructurePose;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructureSubsystem;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;

public class SuperStructCommandsFactory {
    private final SuperStructureSubsystem superStructureSubsystem;
    private boolean sampleInArm = false;
    public SuperStructCommandsFactory(SuperStructureSubsystem superStructureSubsystem) {
        this.superStructureSubsystem = superStructureSubsystem;
    }

    public Command intakeContinuously(DoubleSupplier extendSpeed, DoubleSupplier intakeRotation, BooleanSupplier pushDownClawButton, BooleanSupplier closeClawButton) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(new InstantCommand(() -> {
            superStructureSubsystem.setIntakeRotate(0);
            sampleInArm = false;
            superStructureSubsystem.setIntakeClaw(false);
        }));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_INTAKE));

        AtomicReference<Double> extendPosition = new AtomicReference<>(0.0);
        Supplier<SuperStructurePose> intakePose = () ->
                (pushDownClawButton.getAsBoolean() ? SuperStructurePose.INTAKE : SuperStructurePose.PREPARE_TO_INTAKE)
                        .withExtendPosition(extendPosition.get());
        Command updateExtendPose = new RunCommand(() -> extendPosition.set(
                MathUtil.clamp(extendPosition.get() + 0.02 * extendSpeed.getAsDouble(),
                0, 1)));
        Command updateRotation = new RunCommand(() -> superStructureSubsystem.setIntakeRotate(intakeRotation.getAsDouble()));
        Command closeClawWhenRequested = new RunCommand(() -> superStructureSubsystem.setIntakeClaw(closeClawButton.getAsBoolean()));
        sequence.addCommands(
                superStructureSubsystem.follow(intakePose)
                        .alongWith(updateExtendPose)
                        .alongWith(updateRotation)
                        .alongWith(closeClawWhenRequested));
        return sequence;
    }

    public Command holdIntake() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(superStructureSubsystem.closeIntakeClaw());
        sequence.addCommands(new ConditionalCommand(
                superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE_COMPLETE)
                        .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(1)),
                new InstantCommand(),
                () -> superStructureSubsystem.extend.getCurrentSetPoint() > 0.1
        ));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.HOLD)
                .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(0)));
        return sequence;
    }

    public Command passSampleToUpperArm() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);

        sequence.addCommands(superStructureSubsystem.closeIntakeClaw());
        sequence.addCommands(new ConditionalCommand(
                superStructureSubsystem.moveToPose(SuperStructurePose.INTAKE_COMPLETE)
                        .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(1)),
                new InstantCommand(),
                () -> superStructureSubsystem.extend.getCurrentSetPoint() > 0.1
        ));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_PASS)
                .alongWith(superStructureSubsystem.openArmClaw())
                .beforeStarting(() -> superStructureSubsystem.setIntakeRotate(0)));

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PASS));

        sequence.addCommands(superStructureSubsystem.closeArmClaw());
        sequence.addCommands(superStructureSubsystem.openIntakeClaw());

        sequence.addCommands(new InstantCommand(() -> sampleInArm = true));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.PREPARE_TO_PASS)
                .alongWith(superStructureSubsystem.openIntakeClaw()));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.HOLD));

        return sequence;
    }

    public Command scoreSample(BooleanSupplier scoreButton, BooleanSupplier goForLowBasket, BooleanSupplier goForHighBasket) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(superStructureSubsystem);
        Command passSampleToArm_ifNoSampleInArm = new ConditionalCommand(
                new InstantCommand(),
                passSampleToUpperArm(),
                () -> sampleInArm);
        sequence.addCommands(passSampleToArm_ifNoSampleInArm);

        final SuperStructurePose prepareToScore = SuperStructurePose.SCORE_SAMPLE.withArmFlipPosition(0.7);
        sequence.addCommands(superStructureSubsystem.moveToPose(prepareToScore));
        AtomicBoolean scoreLowBasket = new AtomicBoolean(false);
        Command updateLowHighBasketChoice = new RunCommand(() -> {
            if (goForLowBasket.getAsBoolean())
                scoreLowBasket.set(true);
            if (goForHighBasket.getAsBoolean())
                scoreLowBasket.set(false);
        });
        Command stayAtScoringHeight = superStructureSubsystem.follow(() -> prepareToScore.withElevatorPosition(
                scoreLowBasket.get() ? 0.6 : 1));
        sequence.addCommands(new WaitUntilCommand(scoreButton)
                .raceWith(updateLowHighBasketChoice)
                .raceWith(stayAtScoringHeight));

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SAMPLE));
        sequence.addCommands(superStructureSubsystem.openArmClaw()
                .beforeStarting(() -> sampleInArm = false));
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SAMPLE.withArmFlipPosition(0.5)));

        return sequence;
    }

    public Command grabSpecimen() {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.HOLD.withElevatorPosition(0.2)));

        sequence.addCommands(superStructureSubsystem.openSpecimenClaw());

        sequence.addCommands(superStructureSubsystem.closeSpecimenClaw());

        return sequence;
    }

    public Command scoreSpecimen(BooleanSupplier scoreButton) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SPECIMEN));

        sequence.addCommands(new WaitUntilCommand(scoreButton));

        sequence.addCommands(superStructureSubsystem.moveToPose(SuperStructurePose.SCORE_SPECIMEN.withElevatorPosition(0.45)));

        sequence.addCommands(superStructureSubsystem.openSpecimenClaw());

        return sequence;
    }

}
