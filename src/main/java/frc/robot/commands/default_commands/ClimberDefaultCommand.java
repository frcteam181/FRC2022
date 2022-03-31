package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberDefaultCommand extends CommandBase {

    private final Climber m_climber;

    private final XboxController m_climberController;

    public ClimberDefaultCommand(Climber climber, XboxController climberController) {
        m_climber = climber;
        m_climberController = climberController;

        addRequirements(climber);
    }

    @Override
    public void execute() {

        double m_leftValue, m_rightValue;
        m_leftValue = m_climberController.getRightY();
        m_rightValue = m_climberController.getLeftY();
        m_climber.move_leftClimber(m_leftValue);
        m_climber.move_rightClimber(m_rightValue);
        
    }
    
}
