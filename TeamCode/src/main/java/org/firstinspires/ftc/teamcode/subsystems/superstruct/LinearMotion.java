package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;

public class LinearMotion implements SimpleMechanism, Subsystem {
    public final String name;

    private final DcMotor[] motors;
    private final boolean[] motorsReversed;
    private final DcMotor encoder;
    private final Optional<BooleanSupplier> optionalLimitSwitch;
    private final boolean encoderReversed;
    private final double maximumExtendingLength;
    private final PIDController controller;
    private final double kG, kV, kS;

    private double setPoint;
    private boolean calibrated = false;

    /**
     * A DCMotor driven linear motion
     *
     * Encoders <strong>MUST</strong> be zeroed before the match
     *
     * @param kG the percent motor power required to balance gravity, 0.0 if the linear motion stays still on its own
     * @param kV the velocity gain in motor power / mechanism velocity
     *          (velocity is in position/second, where position is 0~1)
     * @param kP the proportion gain in motor power / position error (where position is 0~1)
     * @param kS the static gain, or the percent power required to move the mechanism
     * */
    public LinearMotion(
            String name,
            DcMotor[] motors, boolean[] motorsReversed,
            DcMotor encoder, boolean encoderReversed,
            Optional<BooleanSupplier> optionalLimitSwitch,
            double maximumExtendingLength,
            double kG, double kV, double kP, double kS) {
        this.name = name;

        assert motors.length == motorsReversed.length
                : "Motors and reversed list length not equal!";
        this.motors = motors;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.optionalLimitSwitch = optionalLimitSwitch;

        this.maximumExtendingLength = maximumExtendingLength;
        this.controller = new PIDController(kP, 0, 0);
        this.kG = kG;
        this.kV = kV;
        this.kS = kS;

        for (DcMotor motor:motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);

        previousSetPoint = setPoint = 0; // robot must be in starting configuration
    }

    private double previousSetPoint;
    private double encoderZeroPosition = 0.0;
    @Override
    public void periodic() {
        SystemConstants.telemetry.addLine("<-- " + name + " -->");
        SystemConstants.telemetry.addData(name + "SetPoint", setPoint);

        if (optionalLimitSwitch.isPresent() && optionalLimitSwitch.get().getAsBoolean())
            encoderZeroPosition = encoder.getCurrentPosition();
        if (optionalLimitSwitch.isPresent() && optionalLimitSwitch.get().getAsBoolean())
            calibrated = true;

        double currentPosition =(encoder.getCurrentPosition() - encoderZeroPosition) * (encoderReversed ? -1:1) / maximumExtendingLength;
        double desiredVelocity = (setPoint - previousSetPoint) * SystemConstants.ROBOT_UPDATE_RATE_HZ;
        if ((!calibrated && optionalLimitSwitch.isPresent())
                || (setPoint == 0 && currentPosition > 0.01)) {
            runPower(-0.4);
            return;
        }

        previousSetPoint = setPoint;
        double feedForwardPower = desiredVelocity * kV
                + Math.signum(desiredVelocity) * kS
                + kG;
        double feedBackPower = controller.calculate(currentPosition, setPoint);

        runPower(feedForwardPower + feedBackPower);

        SystemConstants.telemetry.addData(name + "CurrentPosition", currentPosition);
        SystemConstants.telemetry.addData(name + "DesiredVelocity", desiredVelocity);
        SystemConstants.telemetry.addData(name + "FFPower", feedForwardPower);
        SystemConstants.telemetry.addData(name + "FBPower", feedBackPower);
    }

    private void runPower(double power) {
        for (int i = 0; i < motors.length; i++)
            motors[i].setPower((power) * (motorsReversed[i] ? -1:1));
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getCurrentSetPoint() {
        return this.setPoint;
    }
}
