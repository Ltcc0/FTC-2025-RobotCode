package org.firstinspires.ftc.teamcode.subsystems.superstruct;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.SuperStructure.ProfiledMechanismSet;

/**
 * The order is:
 * intake base, intake flip,
 * */
public class SuperStructureState implements Cloneable {
    public final double[] positions;

    public SuperStructureState(
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
    public SuperStructureState clone() {
        return new SuperStructureState(positions[0], positions[1], positions[2], positions[3], positions[4]);
    }

    public SuperStructureState withExtendPosition(double extendPosition) {
         SuperStructureState state = clone();
         state.positions[0] = extendPosition;
         return state;
    }
    public SuperStructureState withIntakeBasePosition(double intakeBasePosition) {
        SuperStructureState state = clone();
        state.positions[1] = intakeBasePosition;
        return state;
    }
    public SuperStructureState withIntakeFlipPosition(double intakeFlipPosition) {
        SuperStructureState state = clone();
        state.positions[2] = intakeFlipPosition;
        return state;
    }
    public SuperStructureState withElevatorPosition(double elevatorPosition) {
        SuperStructureState state = clone();
        state.positions[3] = elevatorPosition;
        return state;
    }
    public SuperStructureState withArmFlipPosition(double armFlipPosition) {
        SuperStructureState state = clone();
        state.positions[4] = armFlipPosition;
        return state;
    }

    public void feedToSuperStructure(ProfiledMechanismSet superStructure) {
        superStructure.requestPositions(positions);
    }

    public static final SuperStructureState HOLD = new SuperStructureState(
            0,
            0.5, 0.5,
            0,
            0.05);

    public static final SuperStructureState PREPARE_TO_PASS = new SuperStructureState(
            0,
            0.5, 0.88,
            0,
            0.05);

    public static final SuperStructureState PASS = new SuperStructureState(
            0,
            0.41, 0.87,
            0,
            0.05);

    public static final SuperStructureState PREPARE_TO_INTAKE = new SuperStructureState(
            0,
            0.82, 0,
            0,
            0);

    public static final SuperStructureState INTAKE = new SuperStructureState(
            0,
            0.93, 0.06,
            0,
            0);

    public static final SuperStructureState SCORE_SAMPLE = new SuperStructureState(
            0,
            0.5, 0.5,
            0,
            0.05);
}
