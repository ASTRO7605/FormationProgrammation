// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.SendablePidfTuner;

public class XRPDrivetrain extends SubsystemBase {
    // The XRP has the left and right motors set to
    // channels 0 and 1 respectively
    private final XRPMotor m_leftMotor;
    private final XRPMotor m_rightMotor;

    // The XRP has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;

    private final XRPGyro m_gyro = new XRPGyro();

    // Set up the differential drive controller
    private final DifferentialDrive m_diffDrive;

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    private final Field2d m_robotFieldPose = new Field2d();
    private final SendablePidfTuner avanceTranslationPIDF = new SendablePidfTuner("Avance Translation");
    private final SendablePidfTuner avanceRotationPIDF = new SendablePidfTuner("Avance Rotation");

    /** Creates a new XRPDrivetrain. */
    public XRPDrivetrain() {
        m_leftMotor = new XRPMotor(DriveConstants.kLeftMotorID);
        m_rightMotor = new XRPMotor(DriveConstants.kRightMotorID);

        m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderChannelA, DriveConstants.kLeftEncoderChannelB);
        m_rightEncoder = new Encoder(DriveConstants.kRightEncoderChannelA, DriveConstants.kRightEncoderChannelB);

        m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

        // Use meters as unit for encoder distances
        m_leftEncoder
                .setDistancePerPulse(
                        (Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
        m_rightEncoder
                .setDistancePerPulse(
                        (Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
        resetEncoders();
        resetGyroAngle();

        // Invert right side since motor is flipped
        m_rightMotor.setInverted(true);

        m_poseEstimator = new DifferentialDrivePoseEstimator(DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(getGyroAngleDegrees()),
                getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d());

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            return;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getEstimatedPose, // Robot pose supplier
                this::resetEstimatedPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                      // individual module feedforwards
                new PPLTVController(0.02, DriveConstants.kMaxXSpeedMps), // PPLTVController is the built in path
                                                                         // following controller for differential
                // drive trains
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData("Robot Pose", m_robotFieldPose);
    }

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getLeftDistanceMeters() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistanceMeters() {
        return m_rightEncoder.getDistance();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics
                .toChassisSpeeds(new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        arcadeDrive(speeds.vxMetersPerSecond / DriveConstants.kMaxXSpeedMps,
                speeds.omegaRadiansPerSecond / DriveConstants.kMaxOmegaSpeedRadPS);
    }

    @Override
    public void simulationPeriodic() {
        m_poseEstimator.update(Rotation2d.fromDegrees(getGyroAngleDegrees()), getLeftDistanceMeters(),
                getRightDistanceMeters());

        m_robotFieldPose.setRobotPose(getEstimatedPose());

        SmartDashboard.putNumber("robotSpeed", getRobotRelativeSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("robotAngularSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
    }

    public double getGyroAngleDegrees() {
        return m_gyro.getAngle();
    }

    public void resetBaseMeasurements() {
        resetEncoders();
        resetGyroAngle();
    }

    public void resetEstimatedPose(Pose2d newPose) {
        m_poseEstimator.resetPose(newPose);
    }

    public void resetGyroAngle() {
        m_gyro.reset();
    }

    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public SendablePidfTuner getAvanceTranslationPIDF() {
        return avanceTranslationPIDF;
    }

    public SendablePidfTuner getAvanceRotationPIDF() {
        return avanceRotationPIDF;
    }
}
