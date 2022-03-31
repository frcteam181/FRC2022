package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class horizontalBelt extends CommandBase{

    private final Conveyor m_conveyor;

    public horizontalBelt(Conveyor conveyor) {

        m_conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {

        m_conveyor.horizontalBelt_move(1);

    }

    @Override
    public void end(boolean interrupted) {

        m_conveyor.stop();
        
    }
    
}

