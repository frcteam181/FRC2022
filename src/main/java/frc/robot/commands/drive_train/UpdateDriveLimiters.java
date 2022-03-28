package frc.robot.commands.drive_train;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class UpdateDriveLimiters  extends InstantCommand{

    DriveTrain m_driveTrain;

    public UpdateDriveLimiters(DriveTrain driveTrain) {

        m_driveTrain = driveTrain;

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {
        
        m_driveTrain.updateDriveLimiters();

    }
    
}
