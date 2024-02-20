package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrunToAngle extends PIDCommand {
  /** Creates a new TrunToAngle. */
  
  public TrunToAngle(Swerve m_swerve, double angleTurn) {
    super(
        // The controller that the command will use
        new PIDController(5, 0.0, 0.0),
        // This should return the measurement
        m_swerve::getYawDouble,
        // This should return the setpoint (can also be a constant)
        angleTurn,
        // This uses the output
        output -> m_swerve.drive(new Translation2d(0, 0), -output, true, true),
        m_swerve);
        
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
