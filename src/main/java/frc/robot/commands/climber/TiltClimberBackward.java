package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TiltClimberBackward  extends CommandBase{

    Climber m_climber;

    public TiltClimberBackward(Climber climber) {

        m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.tilt_climber_backwards();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_climber.stop_climber_tilt();

    }
    
}
