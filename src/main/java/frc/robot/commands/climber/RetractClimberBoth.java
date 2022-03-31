package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractClimberBoth extends CommandBase{

    Climber m_climber;

    public RetractClimberBoth(Climber climber) {

        m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.retract_climberRight();
        m_climber.retract_climberLeft();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_climber.stop_climber();

    }
    
}
