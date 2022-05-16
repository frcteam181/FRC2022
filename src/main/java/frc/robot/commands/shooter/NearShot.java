package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class NearShot extends CommandBase{

    Shooter m_shooter;

    public NearShot(Shooter shooter) {

        m_shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        
        m_shooter.nearShot();

    }
    
    @Override
    public void end(boolean interrupted) {
        
        m_shooter.stop();

    }
}
