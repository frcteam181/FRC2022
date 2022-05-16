package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopShooter extends CommandBase {

    Shooter m_shooter;

    public StopShooter(Shooter shooter) {

        m_shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
        m_shooter.stop();

    }
    
}
