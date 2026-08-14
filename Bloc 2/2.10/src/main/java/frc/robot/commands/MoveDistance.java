// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.lib.SendablePidfTuner;
import frc.robot.subsystems.XRPDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveDistance extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final XRPDrivetrain m_driveTrain;
    private PIDController m_pidTranslationController;
    private PIDController m_pidRotationController;

    private final SendablePidfTuner translationPidf;
    private final SendablePidfTuner rotationPidf;

    private final double targetDistanceMeters;
    private Rotation2d targetRotation;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveDistance(XRPDrivetrain drivetrain, double distanceMeters) {
        m_driveTrain = drivetrain;
        m_pidTranslationController = new PIDController(0, 0, 0);
        m_pidRotationController = new PIDController(0, 0, 0);
        m_pidRotationController.enableContinuousInput(-180, 180);
        translationPidf = m_driveTrain.getAvanceTranslationPIDF();
        rotationPidf = m_driveTrain.getAvanceRotationPIDF();
        targetDistanceMeters = distanceMeters;
        targetRotation = m_driveTrain.getEstimatedPose().getRotation();
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
        m_pidRotationController.enableContinuousInput(-180, 180);

        targetRotation = m_driveTrain.getEstimatedPose().getRotation();
        m_driveTrain.resetEstimatedPose(new Pose2d(0, 0, targetRotation));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentPos = m_driveTrain.getEstimatedPose().getX();
        m_driveTrain.arcadeDrive(m_pidTranslationController.calculate(currentPos, targetDistanceMeters),
                -m_pidRotationController.calculate(m_driveTrain.getEstimatedPose().getRotation().getDegrees(),
                        targetRotation.getDegrees()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_driveTrain.getEstimatedPose().getX()
                - targetDistanceMeters) < DriveConstants.kMoveDistanceToleranceMeters;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
    }
}
