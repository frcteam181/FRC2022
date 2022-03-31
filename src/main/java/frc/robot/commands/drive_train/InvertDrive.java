package frc.robot.commands.drive_train;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class InvertDrive extends CommandBase{ 

    DriveTrain m_driveTrain;

    public InvertDrive(DriveTrain driveTrain) {

        m_driveTrain = driveTrain;

        addRequirements(driveTrain);

    }

    @Override
    public void initialize() {
        
        m_driveTrain.invertDrive();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
