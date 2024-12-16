package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanismSet;

/**
 * The order is:
 * intake base, intake flip,
 * */
public class SuperStructurePose implements Cloneable {
    public final double[] positions;

    public SuperStructurePose(
            double extendPosition,
            double intakeBasePosition, double intakeFlipPosition,
            double elevatorPosition,
            double armFlipPosition) {
        this.positions = new double[] {
                extendPosition,
                intakeBasePosition, intakeFlipPosition,
                elevatorPosition,
                armFlipPosition
        };
    }

    @NonNull
    @Override
    public SuperStructurePose clone() {
        return new SuperStructurePose(positions[0], positions[1], positions[2], positions[3], positions[4]);
    }

    public SuperStructurePose withExtendPosition(double extendPosition) {
         SuperStructurePose state = clone();
         state.positions[0] = extendPosition;
         return state;
    }
    public SuperStructurePose withIntakeBasePosition(double intakeBasePosition) {
        SuperStructurePose state = clone();
        state.positions[1] = intakeBasePosition;
        return state;
    }
    public SuperStructurePose withIntakeFlipPosition(double intakeFlipPosition) {
        SuperStructurePose state = clone();
        state.positions[2] = intakeFlipPosition;
        return state;
    }
    public SuperStructurePose withElevatorPosition(double elevatorPosition) {
        SuperStructurePose state = clone();
        state.positions[3] = elevatorPosition;
        return state;
    }
    public SuperStructurePose withArmFlipPosition(double armFlipPosition) {
        SuperStructurePose state = clone();
        state.positions[4] = armFlipPosition;
        return state;
    }

    public void feedToSuperStructure(ProfiledMechanismSet superStructure) {
        superStructure.requestPositions(positions);
    }

    public static final SuperStructurePose PREPARE_TO_INTAKE = new SuperStructurePose(
            0,
            0.83, 0,
            0,
            0);

    public static final SuperStructurePose INTAKE = new SuperStructurePose(
            0,
            0.96, 0.06,
            0,
            0);

    public static final SuperStructurePose INTAKE_COMPLETE = new SuperStructurePose(
            0,
            0.9, 0.5,
            0,
            0);

    public static final SuperStructurePose HOLD = new SuperStructurePose(
            0,
            0.5, 0.5,
            0,
            0.05);

    public static final SuperStructurePose PREPARE_TO_PASS = new SuperStructurePose(
            0,
            0.54, 0.92,
            0,
            0.05);

    public static final SuperStructurePose PASS = new SuperStructurePose(
            0,
            0.42, 0.86,
            0,
            0.05);


    public static final SuperStructurePose SCORE_SAMPLE = new SuperStructurePose(
            0,
            0.5, 0.5,
            1,
            0.95);

    public static final SuperStructurePose SCORE_SPECIMEN = new SuperStructurePose(
            0,
            0.5, 0.5,
            0.62,
            0.05);
}
