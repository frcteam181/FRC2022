package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class verticalBelt extends CommandBase{

    private final Conveyor m_conveyor;
    private final XboxController m_operatorController;

    public verticalBelt(Conveyor conveyor, XboxController operatorController) {

        m_conveyor = conveyor;
        m_operatorController = operatorController;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {

        double m_verticalValue;

        m_verticalValue = m_operatorController.getRightY();
        SmartDashboard.putNumber("Vertical Value", m_verticalValue);
        m_conveyor.verticalBelt_move(m_verticalValue);

        //m_conveyor.teleop_operate(m_horizontalValue, m_verticalValue);

    }
    
}
