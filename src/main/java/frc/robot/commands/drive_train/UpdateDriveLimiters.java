package frc.robot.commands.drive_train;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrainTalonSRX;

public class UpdateDriveLimiters  extends InstantCommand{

    DriveTrainTalonSRX m_driveTrain;

    public UpdateDriveLimiters(DriveTrainTalonSRX driveTrain) {

        m_driveTrain = driveTrain;

        addRequirements(m_driveTrain);

    }

    @Override
    public void initialize() {
        
        m_driveTrain.updateDriveLimiters();

    }
    
}
