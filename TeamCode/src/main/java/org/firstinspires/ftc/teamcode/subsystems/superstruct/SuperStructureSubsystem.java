package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanism;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanismSet;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ServoEx;

import java.util.Optional;
import java.util.function.Supplier;

public class SuperStructureSubsystem extends SubsystemBase {
    public static boolean openWide = false;
    public final LinearMotion extend, elevator;
    private final ProfiledMechanismSet superStructure;
    private final Servo intakeRotate, intakeClaw, armClaw;
    private boolean intakeClawClosed, armClawClosed;
    private double intakeRotAngle = 0.0;

    public SuperStructureSubsystem(HardwareMap hardwareMap) {
        this.extend = new LinearMotion(
                "Extend",
                new DcMotor[]{hardwareMap.get(DcMotor.class, "extend")},
                new boolean[] {true},
                hardwareMap.get(DcMotor.class, "extend"),
                true,
                Optional.empty(),
                2000,
                0,
                0.8,
                7.5,
                0.4);
        this.elevator = new LinearMotion(
                "Elevator",
                new DcMotor[]{hardwareMap.get(DcMotor.class, "elevator1"), hardwareMap.get(DcMotor.class, "elevator2")},
                new boolean[] {true, false},
                hardwareMap.get(DcMotor.class, "elevator1"),
                true,
                Optional.empty(),
                600,
                0.1,
                0.4,
                5,
                0.1);
        final ProfiledMechanism intakeBase = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "intakeBase")),
                0.6,
                0.5);
        final ProfiledMechanism intakeFlip = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "intakeFlip")),
                0.4,
                0.5);
        final ProfiledMechanism armFlip = new ProfiledMechanism(
                new ServoEx(hardwareMap.get(Servo.class, "armFlip")),
                0.6,
                0.05);
        this.superStructure = new ProfiledMechanismSet(
                new ProfiledMechanism[]{
                        new ProfiledMechanism(extend, 1.2, 0),
                        intakeBase, intakeFlip,
                        new ProfiledMechanism(elevator, 0.8, 0),
                        armFlip},
                SuperStructurePose.HOLD.positions);

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

    public void requestPose(SuperStructurePose state) {
        superStructure.requestPositions(state.positions);
    }

    @Override
    public void periodic() {
        superStructure.update();
        extend.periodic();
        elevator.periodic();

        intakeClaw.setPosition(intakeClawClosed ? 1 : (openWide ? 0 : 0.3));
        armClaw.setPosition(armClawClosed ? 1 : 0.4);
        intakeRotate.setPosition(0.5 + intakeRotAngle * 0.5);
    }

    public boolean superStructAtReference() {
        return this.superStructure.atReference();
    }

    public Command moveToPose(SuperStructurePose pose) {
        // wait for 100ms for debug
        final Command waitForSuperStructureMovement = new WaitCommand(100)
                .andThen(new WaitUntilCommand(this::superStructAtReference));
        return new InstantCommand(() -> requestPose(pose), this)
                .alongWith(waitForSuperStructureMovement);
    }

    public Command follow(Supplier<SuperStructurePose> pose) {
        return new RunCommand(() -> requestPose(pose.get()), this);
    }

    public Command closeIntakeClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setIntakeClaw(true))
                        .andThen(new WaitCommand(250)),
                new InstantCommand(),
                () -> !intakeClawClosed);
    }

    public Command openIntakeClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setIntakeClaw(false))
                        .andThen(new WaitCommand(250)),
                new InstantCommand(),
                () -> intakeClawClosed);
    }

    public Command closeArmClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setArmClaw(true))
                        .andThen(new WaitCommand(200)),
                new InstantCommand(),
                () -> !armClawClosed);
    }

    public Command openArmClaw() {
        return new ConditionalCommand(
                new InstantCommand(() -> setArmClaw(false))
                        .andThen(new WaitCommand(150)),
                new InstantCommand(),
                () -> armClawClosed);
    }
}