package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TiltClimberForward  extends CommandBase{

    Climber m_climber;

    public TiltClimberForward(Climber climber) {

        m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.tilt_climber_forward();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_climber.stop_climber_tilt();

    }
    
}
