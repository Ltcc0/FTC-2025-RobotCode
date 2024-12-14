package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

import edu.wpi.first.math.controller.PIDController;

public class LinearMotion implements SimpleMechanism, Subsystem {
    private final DcMotor[] motors;
    private final boolean[] motorsReversed;
    private final DcMotor encoder;
    private final boolean encoderReversed;
    private final double maximumExtendingLength;
    private final PIDController controller;
    private final double kG, kV, kS;

    private double setPoint;

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
            DcMotor[] motors, boolean[] motorsReversed,
            DcMotor encoder, boolean encoderReversed,
            double maximumExtendingLength,
            double kG, double kV, double kP, double kS) {
        assert motors.length == motorsReversed.length
                : "Motors and reversed list length not equal!";
        this.motors = motors;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.maximumExtendingLength = maximumExtendingLength;
        this.controller = new PIDController(kP, 0, 0);
        this.kG = kG;
        this.kV = kV;
        this.kS = kS;

        for (DcMotor motor:motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousSetPoint = setPoint = 0; // robot must be in starting configuration
    }

    private double previousSetPoint;
    @Override
    public void periodic() {
        double currentPosition = encoder.getCurrentPosition() / maximumExtendingLength;
        double desiredVelocity = (setPoint - previousSetPoint) / SystemConstants.ROBOT_UPDATE_RATE_HZ;
        previousSetPoint = setPoint;
        double feedForwardPower = desiredVelocity * kV
                + Math.signum(desiredVelocity) * kS
                + kG;
        double feedBackPower = controller.calculate(currentPosition, setPoint);

        for (int i = 0; i < motors.length; i++)
            motors[i].setPower((feedForwardPower + feedBackPower) * (motorsReversed[i] ? -1:1));
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }
}
