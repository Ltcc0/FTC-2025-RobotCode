package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.SuperStructCommandsFactory;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MapleOdometerWheelsOdometry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructureSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer implements AutoCloseable {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;
    public final SuperStructureSubsystem superStructureSubsystem;
    public final SuperStructCommandsFactory superStructCommandsFactory;

    public final MapleOdometerWheelsOdometry odometry;

    // public final AprilTagVision vision;
    /** create all the subsystem with the hardware map */
    public RobotContainer(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
        this.odometry = new MapleOdometerWheelsOdometry(hardwareMap, new Pose2d());
        odometry.register();
        odometry.setDefaultCommand(new FunctionalCommand(
                () -> {},
                () -> SystemConstants.telemetry.addData("Estimated Pose", odometry.getEstimatedPose()),
                (Boolean terminated) -> {},
                () -> false,
                odometry
        ));

        this.driveSubsystem = new MecanumDriveSubsystem(hardwareMap, odometry);
        this.superStructureSubsystem = new SuperStructureSubsystem(hardwareMap);
        this.superStructCommandsFactory = new SuperStructCommandsFactory(superStructureSubsystem);
    }

    @Override
    public void close() throws Exception {
        odometry.close();
    }
}
