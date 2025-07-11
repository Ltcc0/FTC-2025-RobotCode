package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.autos.OneSpecimenPush;
import org.firstinspires.ftc.teamcode.autos.OneSpecimenThreeSampleAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;

@Autonomous(name="<Auto> 1 Specimen + Push")

public class OneSpecimenPushEntrance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotContainer(hardwareMap, AllianceSide.RED),
                new OneSpecimenPush()
        );
        OpModeUtils.runAutoMode(robot, this);
    }
}
