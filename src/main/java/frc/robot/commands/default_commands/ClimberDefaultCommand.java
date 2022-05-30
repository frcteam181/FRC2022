package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberDefaultCommand extends CommandBase {

    private final Climber m_climber;

    private final XboxController m_climberController;

    private double m_leftValue;

    public ClimberDefaultCommand(Climber climber, XboxController climberController) {

        m_climber = climber;
        m_climberController = climberController;

        addRequirements(climber);

    }

    @Override
    public void initialize() {
        m_leftValue = 0;
    }

    @Override
    public void execute() {

        m_leftValue = m_climberController.getRightY();

        m_climber.moveClimber(m_leftValue);
        
    }
    
}
