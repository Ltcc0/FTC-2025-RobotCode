package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "MotorsMatch")
public class MotorsMatch extends OpMode {
    private DcMotor mot0, mot1, mot2, mot3;

    @Override
    public void init() {
        mot0 = hardwareMap.get(DcMotor.class, "mot0"); // br, rev   | right
        mot1 = hardwareMap.get(DcMotor.class, "mot1"); // bl        | left, reversed
        mot2 = hardwareMap.get(DcMotor.class, "mot2"); // fr, rev   | center, reversed
        mot3 = hardwareMap.get(DcMotor.class, "mot3"); // fl        |

        mot0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mot3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double speed = gamepad1.left_stick_y;
        mot0.setPower(gamepad1.a ? speed:0);
        mot1.setPower(gamepad1.b ? speed:0);
        mot2.setPower(gamepad1.x ? speed:0);
        mot3.setPower(gamepad1.y ? speed:0);
        telemetry.addData("mot 0 enc ", mot0.getCurrentPosition());
        telemetry.addData("mot 1 enc ", mot1.getCurrentPosition());
        telemetry.addData("mot 2 enc ", mot2.getCurrentPosition());
        telemetry.addData("mot 3 enc ", mot3.getCurrentPosition());

        telemetry.addData("speed", speed);
    }
}
