package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorDefaultCommand extends CommandBase {

    private final Conveyor m_conveyor;

    private final XboxController m_operatorController;

    public ConveyorDefaultCommand(Conveyor conveyor, XboxController operatorController) {
        m_conveyor = conveyor;
        m_operatorController = operatorController;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {

        double m_verticalValue, m_horizontalValue;
        m_verticalValue = m_operatorController.getLeftY();
        m_horizontalValue = m_operatorController.getRightY();
        m_conveyor.shooter_move(m_verticalValue);
        m_conveyor.horizontalBelt_move(m_horizontalValue);
        m_conveyor.verticalBelt_move(m_verticalValue);

    }
    
}
