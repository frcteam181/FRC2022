package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_train.DriveMM;
import frc.robot.subsystems.DriveTrain;

public class TaxiOnly extends SequentialCommandGroup{

    DriveTrain m_driveTrain;

    public TaxiOnly(DriveTrain driveTrain) {

        m_driveTrain = driveTrain;

        addCommands(new DriveMM(m_driveTrain, 181));

    }
    
}
