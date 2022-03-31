package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimberLeft  extends CommandBase{

    Climber m_climber;
    XboxController m_operatorController;

    public ExtendClimberLeft(Climber climber, XboxController operatorController) {

        m_climber = climber;
        m_operatorController = operatorController;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
        m_climber.extend_climberLeft();

    }

    @Override
    public void end(boolean interrupted) {
        
        m_climber.stop_climber();

    }
    
}
