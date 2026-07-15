package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.ModuleIO;
import frc.robot.Constants.VisionConstants.LimelightIMUModes;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.io.Console;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Base extends SubsystemBase {
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveModule[] m_swerveMods;
    private final CANBus canbus = new CANBus(DriveConstants.kCanivoreBusId);

    private Alliance m_allianceColor = Alliance.Red;

    private final Field2d m_robotField;

    /** radians */
    private double m_gyroOffset = 0.0;
    private boolean m_drivingInFieldRelative = true;

    private Rotation2d robotAngle = Rotation2d.kZero;

    private final SysIdRoutine routine;

    private final SwerveModulePosition[] lastModulePositions;
    private final SwerveModulePosition[] moduleDeltas;

    public Base(ModuleIO[] moduleIOs) {
        m_gyro = new Pigeon2(Constants.DriveConstants.pigeonID, canbus);

        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.setYaw(0);

        m_robotField = new Field2d();

        Optional<RobotConfig> robotConfig;

        try {
            robotConfig = Optional.of(RobotConfig.fromGUISettings());
        } catch (Exception e) {
            robotConfig = Optional.empty();
            e.printStackTrace();
        }

        // front left, front right, back left, back right
        m_swerveMods = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            m_swerveMods[i] = new SwerveModule(moduleIOs[i], i);
        }

        lastModulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition() };

        moduleDeltas = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition() };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.swerveKinematics, getRobotYaw(),
                getLastModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                PoseEstimationConstants.kStateStdDevs,
                PoseEstimationConstants.kVisionStdDevsDefault);

        // during SysId routine, robot will drive straight ahead while staying straight.
        // position and velocity are calculated based on the odometry on the x axis
        // (forwards)
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        voltage -> sysIdMotorsVoltage(voltage),
                        log -> {
                            // get average voltage from modules
                            double avgVoltage = 0;
                            for (int i = 0; i < 4; i++) {
                                avgVoltage += m_swerveMods[i].getAppliedDriveVoltage();
                            }
                            avgVoltage /= 4;

                            log.motor("drive")
                                    .voltage(Voltage.ofBaseUnits(avgVoltage, Volts))
                                    .linearVelocity(MetersPerSecond.of(getRobotRelativeSpeeds().vxMetersPerSecond))
                                    .linearPosition(Meters.of(getPose().getX()));
                        },
                        this));

        SmartDashboard.putData("Robot Measurement", m_robotField);

        AutoBuilder.configure(this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) ->

                driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot
                                                         // given ROBOT RELATIVE
                                                         // ChassisSpeeds. Also optionally
                                                         // outputs individual module
                                                         // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(AutoConstants.driveKP, AutoConstants.driveKI, AutoConstants.driveKD), // Translation
                                                                                                               // PID
                                                                                                               // constants
                        new PIDConstants(AutoConstants.turnKP, AutoConstants.turnKI, AutoConstants.turnKD) // Rotation
                                                                                                           // PID
                                                                                                           // constants
                ),
                robotConfig.get(), // The robot configuration
                this::shouldPathsFlip,
                this // Reference to this subsystem to set requirements
        );
        m_drivingInFieldRelative = true;
        m_gyro.reset();
    }

    public boolean shouldPathsFlip() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(
                m_drivingInFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromRadians(getRobotYaw().getRadians() - m_gyroOffset))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            m_swerveMods[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxModuleSpeed);

        for (int i = 0; i < 4; i++) {
            m_swerveMods[i].setDesiredState(desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = m_swerveMods[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getLastModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = m_swerveMods[i].getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(swerveModuleStates);
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRobotYaw(), getLastModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        m_poseEstimator.resetPosition(getRobotYaw(), getLastModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        m_poseEstimator.resetPosition(getRobotYaw(), getLastModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getRobotYaw() {
        // gyro if real, else swerve deltas in sim
        if (Robot.isReal()) {
            robotAngle = m_gyro.getRotation2d();
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = DriveConstants.swerveKinematics.toTwist2d(moduleDeltas);
            robotAngle = robotAngle.plus(new Rotation2d(twist.dtheta));
        }

        return robotAngle;
    }

    public void setModulesFacingForward() {
        for (var module : m_swerveMods) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        }
    }

    public void changeFieldDrivingMode() {
        m_drivingInFieldRelative = !m_drivingInFieldRelative;
    }

    public void switchToRobotRelative() {
        m_drivingInFieldRelative = false;
    }

    public void switchToFieldRelative() {
        m_drivingInFieldRelative = true;
    }

    public void resetGyroOffset(boolean usePoseEstimator) {
        if (!usePoseEstimator) {
            m_gyroOffset = getRobotYaw().getRadians();
        } else {
            double currentHeading = m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
            double currentGyroAngle = getRobotYaw().getRadians();
            double targetHeading = (m_allianceColor == DriverStation.Alliance.Blue) ? 0
                    : Math.toRadians(180);

            double error = targetHeading - currentHeading;

            m_gyroOffset = currentGyroAngle + error;
        }
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        for (SwerveModule mod : m_swerveMods) {
            mod.setNeutralMode(neutralMode);
        }
    }

    @Override
    public void periodic() {
        // update positions
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = m_swerveMods[moduleIndex].getPosition();
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        m_poseEstimator.update(robotAngle, getLastModulePositions());
        {
            var all = DriverStation.getAlliance();
            if (all.isPresent()) {
                m_allianceColor = all.get();
            }
        }

        for (SwerveModule mod : m_swerveMods) {
            mod.update();
        }

        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("RobotOrientation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("RobotHeading", getHeading().getRadians());

        SmartDashboard.putNumber("CANivore Usage", canbus.getStatus().BusUtilization);
    }

    // modules will go straight forward
    public void sysIdMotorsVoltage(Voltage voltage) {
        double volts = voltage.in(edu.wpi.first.units.Units.Volts);
        Rotation2d angleTarget = Rotation2d.kZero;

        for (SwerveModule mod : m_swerveMods) {
            mod.setVoltageState(volts, angleTarget);
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}