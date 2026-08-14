// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.MoveDistance;
import frc.robot.subsystems.Bras;
import frc.robot.subsystems.LineSensor;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XRPDrivetrain m_xrpDrivetrain = new XRPDrivetrain();
    private final Vision m_vision = new Vision();
    private final Bras m_bras = new Bras();
    private final LineSensor m_lineSensor = new LineSensor();

    private final CommandXboxController m_xboxController;
    private final DigitalInput m_button;

    private final Command autoAvanceCommand = new InstantCommand(() -> {
        m_xrpDrivetrain.resetBaseMeasurements();
    })
            .andThen(new InstantCommand(() -> {
                m_bras.setServoPosition(0.15);
            }))
            .andThen(new WaitCommand(2))
            .andThen(new MoveDistance(m_xrpDrivetrain, 0.35))
            .andThen(new WaitCommand(2))
            .andThen(new InstantCommand(() -> {
                m_bras.setServoPosition(0.85);
            }));

    private final Command autoReculeCommand = new InstantCommand(() -> {
        m_xrpDrivetrain.resetBaseMeasurements();
    })
            .andThen(new InstantCommand(() -> {
                m_bras.setServoPosition(0.15);
            }))
            .andThen(new WaitCommand(2))
            .andThen(new MoveDistance(m_xrpDrivetrain, -0.35))
            .andThen(new WaitCommand(2))
            .andThen(new InstantCommand(() -> {
                m_bras.setServoPosition(0.85);
            }));

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        m_xboxController = new CommandXboxController(OperatorConstants.kXboxControllerID);
        m_button = new DigitalInput(OperatorConstants.kButtonPort);

        // Configure the button bindings
        configureButtonBindings();

        m_xrpDrivetrain.setDefaultCommand(new InstantCommand(
                () -> {
                    double xSpeed = -m_xboxController.getLeftY();
                    double rotateSpeed = -m_xboxController.getLeftX();
                    // if (m_vision.getDistanceMeters() < VisionConstants.kDistanceThresholdMeters
                    // && xSpeed > 0) {
                    // xSpeed = 0;
                    // }
                    m_xrpDrivetrain.arcadeDrive(xSpeed, rotateSpeed);
                },
                m_xrpDrivetrain));

        autoChooser.addOption("Avance", autoAvanceCommand);
        autoChooser.addOption("Recule", autoReculeCommand);
        autoChooser.setDefaultOption("None", new InstantCommand());

        SmartDashboard.putData("Autonomous chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void resetBaseMeasurements() {
        m_xrpDrivetrain.resetBaseMeasurements();
    }
}
