package org.firstinspires.ftc.teamcode.codeEntrances;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.autos.OneSpecimenThreeSampleAuto;
import org.firstinspires.ftc.teamcode.autos.ThreeSample;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;

@Autonomous(name="<Auto> 3 Sample")

public class ThreeSampleEntrance extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotContainer(hardwareMap, AllianceSide.RED),
                new ThreeSample()
        );
        OpModeUtils.runAutoMode(robot, this);
    }
}
