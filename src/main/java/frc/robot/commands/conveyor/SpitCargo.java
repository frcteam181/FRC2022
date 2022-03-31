package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class SpitCargo extends CommandBase{

    Conveyor m_conveyor;

    public SpitCargo(Conveyor conveyor) {

        m_conveyor = conveyor;

        addRequirements(conveyor);

    }

    @Override
    public void initialize() {
        
        m_conveyor.belt_move(1);

    }

    @Override
    public void end(boolean interrupted) {
        
        m_conveyor.belt_move(0);

    }
    
}
