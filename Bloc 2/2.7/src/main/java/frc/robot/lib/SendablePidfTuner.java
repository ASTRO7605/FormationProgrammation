package frc.robot.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SendablePidfTuner implements Sendable {
    private double pGain;
    private double iGain;
    private double dGain;
    private double fGain;

    private final String pGainPreferenceName;
    private final String iGainPreferenceName;
    private final String dGainPreferenceName;
    private final String fGainPreferenceName;

    public SendablePidfTuner(String pidName) {
        pGainPreferenceName = pidName + " P Gain";
        iGainPreferenceName = pidName + " I Gain";
        dGainPreferenceName = pidName + " D Gain";
        fGainPreferenceName = pidName + " F Gain";
        initPreferences();

        SmartDashboard.putData(pidName + " PIDF Gains", this);
    }

    private void initPreferences() {
        Preferences.initDouble(pGainPreferenceName, 0);
        Preferences.initDouble(iGainPreferenceName, 0);
        Preferences.initDouble(dGainPreferenceName, 0);
        Preferences.initDouble(fGainPreferenceName, 0);

        pGain = Preferences.getDouble(pGainPreferenceName, 0);
        iGain = Preferences.getDouble(iGainPreferenceName, 0);
        dGain = Preferences.getDouble(dGainPreferenceName, 0);
        fGain = Preferences.getDouble(fGainPreferenceName, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDF Tuner");
        builder.addDoubleProperty("1. P", () -> {
            return Preferences.getDouble(pGainPreferenceName, 0);
        }, (double newVal) -> {
            Preferences.setDouble(pGainPreferenceName, newVal);
            pGain = newVal;
        });
        builder.addDoubleProperty("2. I", () -> {
            return Preferences.getDouble(iGainPreferenceName, 0);
        }, (double newVal) -> {
            Preferences.setDouble(iGainPreferenceName, newVal);
            iGain = newVal;
        });
        builder.addDoubleProperty("3. D", () -> {
            return Preferences.getDouble(dGainPreferenceName, 0);
        }, (double newVal) -> {
            Preferences.setDouble(dGainPreferenceName, newVal);
            dGain = newVal;
        });
        builder.addDoubleProperty("4. F", () -> {
            return Preferences.getDouble(fGainPreferenceName, 0);
        }, (double newVal) -> {
            Preferences.setDouble(fGainPreferenceName, newVal);
            fGain = newVal;
        });
    }

    public double getPGain() {
        return pGain;
    }

    public double getIGain() {
        return iGain;
    }

    public double getDGain() {
        return dGain;
    }

    public double getFGain() {
        return fGain;
    }
}
