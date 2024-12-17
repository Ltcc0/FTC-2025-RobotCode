package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveFactory;
import org.firstinspires.ftc.teamcode.subsystems.superstruct.SuperStructureSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;
    private final Runnable calibrateOdometry;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

        this.calibrateOdometry = () -> robotContainer.driveSubsystem.setPose(new Pose2d());
        calibrateOdometry.run();
        configureKeyBindings();
        SuperStructureSubsystem.openWide = false;
    }

    private void configureKeyBindings() {
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenHeld(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

//        robotContainer.driveSubsystem.setDefaultCommand(JoystickDriveFactory.joystickDrive(
//                robotContainer.driveSubsystem,
//                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
//                () -> -pilotGamePad.getRightY(),
//                () -> -pilotGamePad.getRightX()));

        robotContainer.driveSubsystem.setDefaultCommand(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.START).whenPressed(calibrateOdometry);

        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robotContainer.superStructCommandsFactory.intakeContinuously(
                        copilotGamePad::getLeftY,
                        copilotGamePad::getRightX,
                        copilotGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)::get,
                        () -> copilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5));
        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(robotContainer.superStructCommandsFactory.holdIntake());
        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(robotContainer.superStructCommandsFactory.passSampleToUpperArm());
        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(robotContainer.superStructCommandsFactory.scoreSample(
                        () -> pilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5,
                        copilotGamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)::get,
                        copilotGamePad.getGamepadButton(GamepadKeys.Button.DPAD_UP)::get));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                robotContainer.superStructCommandsFactory.holdIntake());
        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                robotContainer.superStructCommandsFactory.grabSpecimen());

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                robotContainer.superStructCommandsFactory.scoreSpecimen(
                        () -> pilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5));
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
