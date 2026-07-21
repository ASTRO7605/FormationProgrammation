// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.XRPDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SuivreVision extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final XRPDrivetrain m_driveTrain;
    private final Vision m_vision;
    private double m_initialPoseLeft;
    private double m_initialPoseRight;
    private double m_initialOrientation;
    private PIDController m_pidTranslationController;
    private PIDController m_pidRotationController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SuivreVision(XRPDrivetrain drivetrain, Vision vision) {
        m_driveTrain = drivetrain;
        m_vision = vision;
        m_pidTranslationController = new PIDController(0, 0, 0);
        m_pidRotationController = new PIDController(0, 0, 0);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidTranslationController.setP(SmartDashboard.getNumber("P translation gain", 0));
        m_pidRotationController.setP(SmartDashboard.getNumber("P rotation gain", 0));
        m_driveTrain.resetEstimatedPose(new Pose2d(0, 0, new Rotation2d(Math.PI / 2)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(-m_pidTranslationController.calculate(m_vision.getDistanceMeters(), 0.2),
                m_pidRotationController.calculate(m_driveTrain.getEstimatedPose().getRotation().getDegrees(),
                        90));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return (m_driveTrain.getLeftDistanceMeters() >= m_initialPoseLeft + 1)
        // && (m_driveTrain.getRightDistanceMeters() >= m_initialPoseRight + 1);
        return false;
    }
}
