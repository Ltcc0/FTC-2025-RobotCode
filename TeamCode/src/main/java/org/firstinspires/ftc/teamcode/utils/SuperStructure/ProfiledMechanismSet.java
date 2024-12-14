package org.firstinspires.ftc.teamcode.utils.SuperStructure;

public class ProfiledMechanismSet {
    private final ProfiledMechanism[] profiledMechanisms;

    public ProfiledMechanismSet(ProfiledMechanism[] profiledMechanisms, double[] initialPositions) {
        assert profiledMechanisms.length == initialPositions.length
                : "Mechanisms and Positions Length Not Match";
        this.profiledMechanisms = profiledMechanisms;
    }

    public void requestPositions(double[] setPoints) {
        assert setPoints.length == profiledMechanisms.length
                : "Mechanisms and Positions Length Not Match";
       for (int i = 0; i < setPoints.length; i++)
           profiledMechanisms[i].requestPosition(setPoints[i]);
    }

    public void update() {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            mechanism.update();
    }
    public void update(double dt) {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            mechanism.update(dt);
    }

    public boolean atReference() {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            if (!mechanism.atReference()) return false;
        return true;
    }
}
