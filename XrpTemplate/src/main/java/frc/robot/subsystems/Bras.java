package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BrasConstants;

public class Bras extends SubsystemBase {
    private final XRPServo m_servoBras;

    public Bras() {
        m_servoBras = new XRPServo(BrasConstants.kServoID);
        m_servoBras.setPosition(0.5);
    }

    @Override
    public void simulationPeriodic() {
        // SmartDashboard.putNumber("Servo Position", getServoPosition());
        // SmartDashboard.putNumber("Servo Angle", getServoAngle());
    }

    public double getServoPosition() {
        return m_servoBras.getPosition();
    }

    public double getServoAngle() {
        return m_servoBras.getAngle();
    }

    public void setServoPosition(double position) {
        m_servoBras.setPosition(position);
    }

    public void setServoAngle(double angle) {
        m_servoBras.setAngle(angle);
    }
}
