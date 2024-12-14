package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanism;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledSuperStructure;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.ServoEx;

public class SuperStructureSubsystem extends SubsystemBase {
    private final LinearMotion extend, elevator;
    private final ProfiledSuperStructure superStructure;
    private final ServoEx intakeRot, intakeClaw, armClaw;
    private SuperStructureState setPoint;


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
                ;
        final ProfiledMechanism intakeFlip = new ServoEx(hardwareMap.get(Servo.class, "intakeFlip"));
        final ProfiledMechanism armFlip = new ServoEx(hardwareMap.get(Servo.class, "armFlip"));
        this.superStructure = new ProfiledSuperStructure(
                new ProfiledMechanism[]{
                        extend,
                        intakeBase,
                },
                SuperStructureState.HOLD.positions
        );

        this.intakeRot = new ServoEx();
        this.intakeClaw = new ServoEx();
    }
}