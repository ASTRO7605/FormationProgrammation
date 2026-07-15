package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SimSwerveModule implements ModuleIO {
    private final DCMotorSim m_angleMotor;
    private final DCMotorSim m_driveMotor;

    private final PIDController driveVelocityController;
    private final PIDController anglePositionController;

    private double driveVoltage = 0;
    private double angleVoltage = 0;

    private boolean isDriveInOpenLoop = false;
    private boolean isAngleInOpenLoop = false;

    private double calculatedDriveFF = 0;

    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Constants.DriveConstants.driveKS, Constants.DriveConstants.driveKV, Constants.DriveConstants.driveKA);

    public SimSwerveModule(SwerveModuleConstants moduleConstants) {
        /* Angle Motor Config */
        m_angleMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.kSimAngleMOI,
                        DriveConstants.angleGearRatio),
                DCMotor.getKrakenX60(1));
        /* Drive Motor Config */
        m_driveMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.kSimDriveMOI,
                        DriveConstants.driveGearRatio),
                DCMotor.getKrakenX60(1));

        driveVelocityController = new PIDController(DriveConstants.driveKP, DriveConstants.driveKI,
                DriveConstants.driveKD);
        anglePositionController = new PIDController(DriveConstants.angleKP, DriveConstants.angleKI,
                DriveConstants.angleKD);

        anglePositionController.enableContinuousInput(-0.5, 0.5);
    }

    public ModuleIOState update() {
        if (!isDriveInOpenLoop) {
            driveVoltage = calculatedDriveFF
                    + driveVelocityController.calculate(m_driveMotor.getAngularVelocityRPM() / 60.0);
        } else {
            driveVelocityController.reset();
        }

        if (!isAngleInOpenLoop) {
            angleVoltage = anglePositionController.calculate(m_angleMotor.getAngularPositionRotations());
        } else {
            anglePositionController.reset();
        }

        m_driveMotor.setInputVoltage(MathUtil.clamp(driveVoltage, -12.0, 12.0));
        m_angleMotor.setInputVoltage(MathUtil.clamp(angleVoltage, -12.0, 12.0));

        m_driveMotor.update(0.02);
        m_angleMotor.update(0.02);

        return getIOState();
    }

    private ModuleIOState getIOState() {
        var state = new ModuleIOState();

        state.drivePositionRotations = m_driveMotor.getAngularPositionRotations();
        state.driveVelocityRps = m_driveMotor.getAngularVelocityRPM() / 60.0;
        state.driveAppliedVoltage = driveVoltage;

        state.angleAbsolutePosition = Rotation2d.fromRotations(m_angleMotor.getAngularPositionRotations());
        state.anglePosition = Rotation2d.fromRotations(m_angleMotor.getAngularPositionRotations());
        state.angleVelocityRps = m_angleMotor.getAngularVelocityRPM() / 60.0;
        state.angleAppliedVoltage = angleVoltage;

        return state;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {

    }

    public void setDriveVoltage(double voltage) {
        isDriveInOpenLoop = true;
        driveVoltage = voltage;
    }

    public void setDriveVelocityRps(double velocity) {
        isDriveInOpenLoop = false;
        calculatedDriveFF = m_driveFeedForward
                .calculate(Conversions.RPSToMPS(velocity, DriveConstants.wheelCircumference));
        driveVelocityController.setSetpoint(velocity);
    }

    public void setAngleVoltage(double voltage) {
        isAngleInOpenLoop = true;
        angleVoltage = voltage;
    }

    public void setAnglePosition(Rotation2d position) {
        isAngleInOpenLoop = false;
        anglePositionController.setSetpoint(position.getRotations());
    }
}
