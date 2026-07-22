// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

    @Override
    public void simulationPeriodic() {
        m_poseEstimator.update(Rotation2d.fromDegrees(getGyroAngleDegrees()), getLeftDistanceMeters(),
                getRightDistanceMeters());

        m_robotFieldPose.setRobotPose(getEstimatedPose());
        SmartDashboard.putNumber("Gyro Angle Degrees", getGyroAngleDegrees());
        SmartDashboard.putNumber("Left Motor Distance Meters",
                getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Motor Distance Meters",
                getRightDistanceMeters());
        // SmartDashboard.putNumber("Robot Orientation",
        // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        // SmartDashboard.putNumber("Y Position",
        // m_poseEstimator.getEstimatedPosition().getY());
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
}
