package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    public static class ModuleIOState {
        public double drivePositionRotations = 0;
        public double driveVelocityRps = 0;
        public double driveAppliedVoltage = 0;

        public Rotation2d angleAbsolutePosition = Rotation2d.kZero;
        public Rotation2d anglePosition = Rotation2d.kZero;
        public double angleVelocityRps = 0;
        public double angleAppliedVoltage = 0;
    }

    ModuleIOState update();

    public void setNeutralMode(NeutralModeValue neutralMode);

    public void setDriveVoltage(double voltage);

    public void setDriveVelocityRps(double velocity);

    public void setAngleVoltage(double voltage);

    public void setAnglePosition(Rotation2d position);

}
