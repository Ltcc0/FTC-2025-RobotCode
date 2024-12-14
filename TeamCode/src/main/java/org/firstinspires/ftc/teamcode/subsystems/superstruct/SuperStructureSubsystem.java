package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanism;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanismSet;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ServoEx;

public class SuperStructureSubsystem extends SubsystemBase {
    private final LinearMotion extend, elevator;
    private final ProfiledMechanismSet superStructure;
    private final Servo intakeRotate, intakeClaw, armClaw;
    private boolean intakeClawClosed, armClawClosed;
    private double intakeRotAngle = 0.0;

    public SuperStructureSubsystem(HardwareMap hardwareMap) {
        this.extend = new LinearMotion(
                new DcMotor[]{hardwareMap.get(DcMotor.class, "extend")},
                new boolean[] {true},
                hardwareMap.get(DcMotor.class, "extend"),
                true,
                2000,
                0,
                0.8,
                3.5,
                0.1);
        this.elevator = new LinearMotion(
                new DcMotor[]{hardwareMap.get(DcMotor.class, "elevator1"), hardwareMap.get(DcMotor.class, "elevator2")},
                new boolean[] {true, false},
                hardwareMap.get(DcMotor.class, "elevator2"),
                false,
                1000,
                0.05,
                0.5,
                2.5,
                0.05);
        final ProfiledMechanism intakeBase = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "intakeBase")),
                0.8,
                0.5);
        final ProfiledMechanism intakeFlip = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "intakeFlip")),
                0.5,
                0.5);
        final ProfiledMechanism armFlip = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "armFlip")),
                0.8,
                0.05);
        this.superStructure = new ProfiledMechanismSet(
                new ProfiledMechanism[]{
                        new ProfiledMechanism(extend, 1.2, 0),
                        intakeBase, intakeFlip,
                        new ProfiledMechanism(elevator, 0.8, 0),
                        armFlip
                },
                SuperStructureState.HOLD.positions
        );

        this.intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        this.intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        this.armClaw = hardwareMap.get(Servo.class, "armClaw");

        this.intakeClawClosed = false;
        this.armClawClosed = false;
    }

    public void setIntakeClaw(boolean closed) {
        this.intakeClawClosed = closed;
    }

    public void setArmClaw(boolean closed) {
        this.armClawClosed = closed;
    }

    /**
     * @param angle from -1 ~ 1
     * */
    public void setIntakeRotate(double angle) {
        this.intakeRotAngle = angle;
    }

    public void requestState(SuperStructureState state) {
        superStructure.requestPositions(state.positions);
    }

    @Override
    public void periodic() {
        superStructure.update();
        extend.periodic();
        elevator.periodic();

        intakeClaw.setPosition(intakeClawClosed ? 0.8 : 0);
        armClaw.setPosition(armClawClosed ? 0.6 : 0);
        intakeRotate.setPosition(0.5 + intakeRotAngle * 0.5);
    }

    public boolean superStructAtReference() {
        return this.superStructure.atReference();
    }

    public Command moveToState(SuperStructureState state) {
        // wait for 100ms for debug
        final Command waitForSuperStructureMovement = new WaitCommand(100)
                .andThen(new WaitUntilCommand(this::superStructAtReference));
        return new InstantCommand(() -> requestState(state))
                .andThen(waitForSuperStructureMovement);
    }

    public Command closeIntakeClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setIntakeClaw(true))
                        .andThen(new WaitCommand(500)),
                new InstantCommand(),
                () -> !intakeClawClosed);
    }

    public Command openIntakeClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setIntakeClaw(false))
                        .andThen(new WaitCommand(500)),
                new InstantCommand(),
                () -> intakeClawClosed);
    }

    public Command closeArmClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setArmClaw(true))
                        .andThen(new WaitCommand(500)),
                new InstantCommand(),
                () -> !armClawClosed);
    }

    public Command openArmClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setArmClaw(true))
                        .andThen(new WaitCommand(500)),
                new InstantCommand(),
                () -> armClawClosed);
    }
}