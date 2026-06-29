package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class KrakenSwerveModule implements ModuleIO {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_angleCANcoder;

    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Constants.DriveConstants.driveKS, Constants.DriveConstants.driveKV, Constants.DriveConstants.driveKA);

    /* drive motor control requests */
    private final VelocityVoltage m_driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage m_anglePosition = new PositionVoltage(0);

    public KrakenSwerveModule(SwerveModuleConstants moduleConstants) {
        final var canbus = new CANBus(DriveConstants.kCanivoreBusId);

        /* Angle Encoder Config */
        m_angleCANcoder = new CANcoder(moduleConstants.cancoderID, canbus);
        CANcoderConfiguration cancoderConfig = Robot.ctreConfigs.swerveCANcoderConfig.clone();
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.angleOffset.getRotations();
        m_angleCANcoder.getConfigurator().apply(cancoderConfig);

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, canbus);
        m_angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, canbus);
        m_driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.getConfigurator().setPosition(0.0);

        setNeutralMode(NeutralModeValue.Brake);
    }

    private void resetToAbsolute() {
        m_angleMotor.setPosition(m_angleCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public ModuleIOState update() {
        return getIOState();
    }

    private ModuleIOState getIOState() {
        var state = new ModuleIOState();

        state.drivePositionRotations = m_driveMotor.getPosition().getValueAsDouble();
        state.driveVelocityRps = m_driveMotor.getVelocity().getValueAsDouble();
        state.driveAppliedVoltage = m_driveMotor.getMotorVoltage().getValueAsDouble();

        state.angleAbsolutePosition = Rotation2d
                .fromRotations(m_angleCANcoder.getAbsolutePosition().getValueAsDouble());
        state.anglePosition = Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble());
        state.angleVelocityRps = m_angleMotor.getVelocity().getValueAsDouble();
        state.angleAppliedVoltage = m_angleMotor.getMotorVoltage().getValueAsDouble();

        return state;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_driveMotor.setNeutralMode(neutralMode);
        m_angleMotor.setNeutralMode(neutralMode);
    }

    public void setDriveVoltage(double voltage) {
        m_driveMotor.setVoltage(voltage);
    }

    public void setDriveVelocityRps(double velocity) {
        m_driveMotor.setControl(
                m_driveVelocity.withVelocity(velocity)
                        .withFeedForward(m_driveFeedForward.calculate(velocity)));
    }

    public void setAngleVoltage(double voltage) {
        m_angleMotor.setVoltage(voltage);
    }

    public void setAnglePosition(Rotation2d position) {
        m_angleMotor.setControl(m_anglePosition.withPosition(position.getRotations()));
    }
}