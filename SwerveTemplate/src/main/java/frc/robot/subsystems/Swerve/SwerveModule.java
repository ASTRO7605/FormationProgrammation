package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.ModuleIO.ModuleIOState;
import frc.lib.math.Conversions;

public class SwerveModule {
    private final ModuleIO m_moduleIO;
    private final int m_moduleNumber;

    private ModuleIOState m_lastState;

    public SwerveModule(ModuleIO moduleIO, int moduleNumber) {
        m_moduleIO = moduleIO;
        m_moduleNumber = moduleNumber;

        m_lastState = new ModuleIOState();
    }

    public void update() {
        m_lastState = m_moduleIO.update();

        SmartDashboard.putNumber("Mod " + m_moduleNumber + " CANcoder", m_lastState.angleAbsolutePosition.getDegrees());
        SmartDashboard.putNumber("Mod " + m_moduleNumber + " Angle", m_lastState.anglePosition.getDegrees());
        SmartDashboard.putNumber("Mod " + m_moduleNumber + " Position",
                Conversions.rotationsToMeters(m_lastState.drivePositionRotations, DriveConstants.wheelCircumference));
        SmartDashboard.putNumber("Mod " + m_moduleNumber + " Velocity",
                Conversions.RPSToMPS(m_lastState.driveVelocityRps, DriveConstants.wheelCircumference));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getState().angle);
        m_moduleIO.setAnglePosition(desiredState.angle);
        m_moduleIO.setDriveVelocityRps(Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                Constants.DriveConstants.wheelCircumference));
    }

    public void setVoltageState(double driveVoltage, Rotation2d angle) {
        m_moduleIO.setAnglePosition(angle);
        m_moduleIO.setDriveVoltage(driveVoltage);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(m_lastState.driveVelocityRps, Constants.DriveConstants.wheelCircumference),
                m_lastState.anglePosition);
    }

    public double getAppliedDriveVoltage() {
        return m_lastState.driveAppliedVoltage;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Conversions.rotationsToMeters(m_lastState.drivePositionRotations,
                Constants.DriveConstants.wheelCircumference), m_lastState.anglePosition);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_moduleIO.setNeutralMode(neutralMode);
    }
}
