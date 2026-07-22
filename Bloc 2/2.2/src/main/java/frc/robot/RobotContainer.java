// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.MoveUntilObject;
import frc.robot.subsystems.Bras;
import frc.robot.subsystems.LineSensor;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        m_xboxController.x().whileTrue(new MoveUntilObject(m_xrpDrivetrain, m_vision, m_bras));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return m_autoCommand;
        return new InstantCommand();
    }

    public void resetBaseMeasurements() {
        m_xrpDrivetrain.resetBaseMeasurements();
    }
}
