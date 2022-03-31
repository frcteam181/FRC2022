package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimberBoth extends CommandBase{

    Climber m_climber;

    public ExtendClimberBoth(Climber climber) {

        m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.extend_climberRight();
        m_climber.extend_climberLeft();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_climber.stop_climber();

    }
    
}
