package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.autos.FiveSpecimenAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;

// @Autonomous(name="<Auto> 5 Specimen")
public class FiveSpecimenAutoEntrance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotContainer(hardwareMap, AllianceSide.RED),
                new FiveSpecimenAuto()
        );
        OpModeUtils.runAutoMode(robot, this);
    }
}
