package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class AutoClimbUp  extends CommandBase{

    Climber m_climber;

    public AutoClimbUp(Climber climber) {

        m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.tilt_climber_full_backwards();

    }



    @Override
    public void end(boolean interrupted) {

    }
    
}
