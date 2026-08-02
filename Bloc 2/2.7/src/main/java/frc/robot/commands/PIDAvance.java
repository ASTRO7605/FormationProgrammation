// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.lib.SendablePidfTuner;
import frc.robot.subsystems.XRPDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PIDAvance extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final XRPDrivetrain m_driveTrain;
    private PIDController m_pidTranslationController;
    private PIDController m_pidRotationController;

    private final SendablePidfTuner translationPidf;
    private final SendablePidfTuner rotationPidf;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PIDAvance(XRPDrivetrain drivetrain) {
        m_driveTrain = drivetrain;
        m_pidTranslationController = new PIDController(0, 0, 0);
        m_pidRotationController = new PIDController(0, 0, 0);
        translationPidf = m_driveTrain.getAvanceTranslationPIDF();
        rotationPidf = m_driveTrain.getAvanceRotationPIDF();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidTranslationController.setPID(translationPidf.getPGain(), translationPidf.getIGain(),
                translationPidf.getDGain());
        m_pidRotationController.setPID(rotationPidf.getPGain(), rotationPidf.getIGain(),
                rotationPidf.getDGain());
        m_driveTrain.resetEstimatedPose(new Pose2d(0, 0, new Rotation2d(Math.PI / 2)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentPos = m_driveTrain.getEstimatedPose().getY();
        m_driveTrain.arcadeDrive(m_pidTranslationController.calculate(currentPos, 0.5),
                m_pidRotationController.calculate(m_driveTrain.getEstimatedPose().getRotation().getDegrees(),
                        90));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
    }
}
