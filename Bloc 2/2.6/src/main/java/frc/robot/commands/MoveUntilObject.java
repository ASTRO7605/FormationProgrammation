package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Bras;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.XRPDrivetrain;

public class MoveUntilObject extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final XRPDrivetrain m_driveTrain;
    private final Vision m_vision;
    private final Bras m_bras;

    private boolean useArm;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveUntilObject(XRPDrivetrain drivetrain, Vision vision, Bras bras) {
        m_driveTrain = drivetrain;
        m_vision = vision;
        m_bras = bras;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain, bras);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_bras.setServoPosition(0.1);

        System.out.println("init");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(0.75, 0);
        System.out.println("execute");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_vision.getDistanceMeters() < 0.2) {
            System.out.println("finished true");
            return true;
        }
        System.out.println("finished false");
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.arcadeDrive(0, 0);
        if (!interrupted) {
            m_bras.setServoPosition(0.9);
        }
        System.out.println("end");
        System.out.println("");
    }
}
