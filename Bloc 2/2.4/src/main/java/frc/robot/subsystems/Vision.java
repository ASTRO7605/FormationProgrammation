package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final AnalogInput m_capteurDistance;

    public Vision() {
        m_capteurDistance = new AnalogInput(VisionConstants.kCapteurID);
        m_capteurDistance.setAverageBits(VisionConstants.kAverageBits);
    }

    public double getDistanceMeters() {
        // mm to m
        return (double) m_capteurDistance.getAverageValue() / 1000;
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("Distance Vision Meters", getDistanceMeters());
    }

}
