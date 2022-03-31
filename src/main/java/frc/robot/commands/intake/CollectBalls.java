package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class CollectBalls extends CommandBase{

    Intake m_intake;

    /** Creates a new EjectBalls. */
    public CollectBalls(Intake intake) {
        m_intake = intake;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
      m_intake.collectBalls();
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intake.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}

