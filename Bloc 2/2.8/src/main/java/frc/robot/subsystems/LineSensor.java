package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LineSensorConstants;

public class LineSensor extends SubsystemBase {
    private final AnalogInput m_leftLineSensor;
    private final AnalogInput m_rightLineSensor;

    public LineSensor() {
        m_leftLineSensor = new AnalogInput(LineSensorConstants.kLeftSensorID);
        m_rightLineSensor = new AnalogInput(LineSensorConstants.kRightSensorID);

        m_leftLineSensor.setAverageBits(LineSensorConstants.kAverageBits);
        m_rightLineSensor.setAverageBits(LineSensorConstants.kAverageBits);
    }

    @Override
    public void simulationPeriodic() {
        // SmartDashboard.putNumber("Left Line Sensor White Percent",
        // getLeftLineSensorWhitePercent());
        // SmartDashboard.putNumber("Right Line Sensor White Percent",
        // getRightLineSensorWhitePercent());

        // SmartDashboard.putNumber("Left Line Sensor Voltage",
        // getLeftLineSensorVoltage());
        // SmartDashboard.putNumber("Right Line Sensor Voltage",
        // getRightLineSensorVoltage());
    }

    private double getLeftLineSensorVoltage() {
        return m_leftLineSensor.getVoltage();
    }

    private double getRightLineSensorVoltage() {
        return m_rightLineSensor.getVoltage();
    }

    public double getLeftLineSensorWhitePercent() {
        return getLeftLineSensorVoltage() * LineSensorConstants.kConversionFactorPercentWhite;
    }

    public double getRightLineSensorWhitePercent() {
        return getRightLineSensorVoltage() * LineSensorConstants.kConversionFactorPercentWhite;
    }
}
