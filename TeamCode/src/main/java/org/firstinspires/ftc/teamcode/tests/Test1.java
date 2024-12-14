package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test 1")
public class Test1 extends OpMode {
    private DcMotor extend;
    private DcMotor elevator1;
    private DcMotor elevator2;

    private Servo intakeBase;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeFlip;
    private Servo armFlip;
    private Servo armClaw;

    enum State {
        GRAB,
        HOLD,
        PASS,
        SCORE
    }
    private State state;
    private boolean closeClaw;

    @Override
    public void init() {
        extend = hardwareMap.get(DcMotor.class, "extend");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotor.class, "elevator2");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeBase = hardwareMap.get(Servo.class, "intakeBase");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeFlip = hardwareMap.get(Servo.class, "intakeFlip");
        armFlip = hardwareMap.get(Servo.class, "armFlip");
        armClaw = hardwareMap.get(Servo.class, "armClaw");
    }

    @Override
    public void loop() {
        extend.setPower(gamepad2.left_stick_y);

        elevator1.setPower(gamepad2.right_stick_y);
        elevator2.setPower(gamepad2.right_stick_y);

        telemetry.addData("elevator 1 position", elevator1.getCurrentPosition());
        telemetry.addData("elevator 2 position", elevator2.getCurrentPosition());

        telemetry.addData("extend position", extend.getCurrentPosition());

        double intakeBasePosition = 0.5 + 0.5 * gamepad1.left_stick_y;
        intakeBase.setPosition(intakeBasePosition);
        double intakeFlipPosition = 0.5 + 0.5 * gamepad1.right_stick_y;
        intakeFlip.setPosition(intakeFlipPosition);
        double intakeRotatePosition = 0.5 + 0.5 * gamepad1.left_stick_x;
        intakeRotate.setPosition(intakeRotatePosition);
        double intakeClawPosition = gamepad1.right_trigger;
        intakeClaw.setPosition(intakeClawPosition);
        double armFlipPosition = gamepad1.left_bumper ? 1 : 0.05;
        armFlip.setPosition(armFlipPosition);
        double armClawPosition = 1-gamepad1.left_trigger;
        armClaw.setPosition(armClawPosition);

        telemetry.addData("intakeBasePosition", intakeBasePosition);
        telemetry.addData("intakeFlipPosition", intakeFlipPosition);
        telemetry.addData("intakeRotatePosition", intakeRotatePosition);
        telemetry.addData("intakeClawPosition", intakeClawPosition);
        telemetry.addData("armFlipPosition", armFlipPosition);
        telemetry.addData("armClawPosition", armClawPosition);


        /*
        * prepare to intake:
        *   base:0.82
        *   flip:0
        *
        * grab:
        *   base:0.93
        *   flip: 0.06
        *
        * pass:
        *   base:0.38
        *   flip:0.83
        *
        * amm:
        *   arm-flip: 0.05 down
        *  */
    }
}
