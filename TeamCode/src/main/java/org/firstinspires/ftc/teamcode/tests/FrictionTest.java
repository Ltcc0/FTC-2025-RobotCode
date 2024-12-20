package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// @TeleOp(name = "FrictionTest")
public class FrictionTest extends OpMode {
    private DcMotor mot0, mot1, mot2, mot3;

    @Override
    public void init() {
        mot0 = hardwareMap.get(DcMotor.class, "frontLeft");
        mot1 = hardwareMap.get(DcMotor.class, "frontRight");
        mot2 = hardwareMap.get(DcMotor.class, "backLeft");
        mot3 = hardwareMap.get(DcMotor.class, "backRight");

        mot0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double speed = gamepad1.left_stick_y * 0.2;
        mot0.setPower(speed);
        mot1.setPower(speed);
        mot2.setPower(speed);
        mot3.setPower(speed);

        telemetry.addData("speed", speed);
    }
}
