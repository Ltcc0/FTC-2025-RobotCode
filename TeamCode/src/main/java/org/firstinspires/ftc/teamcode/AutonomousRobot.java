package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.autos.Auto;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructureSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleLoopClock;

/**
 * robot during autonomous period
 * here we schedule the autonomous commands
 * */
public class AutonomousRobot extends Robot {
    private static final MapleLoopClock beforeStartPeriodicClock = new MapleLoopClock(24);
    private final Auto auto;
    private final RobotContainer robotContainer;
    public AutonomousRobot(RobotContainer robotContainer, Auto auto) {
        super();
        this.auto = auto;
        this.robotContainer = robotContainer;

        SuperStructureSubsystem.openWide = false;
    }

    public void beforeStartPeriodic() {
        auto.beforeStartPeriodic();
        beforeStartPeriodicClock.tick();
    }

    public void startAuto() {
        super.schedule(auto.getAutonomousCommands(robotContainer));
    }

    @Override
    public void reset() {
        super.reset();
        try {
            robotContainer.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
