package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.subsystems.DriveTrainTalonSRX;

public class MotionMagicDriveTest extends SequentialCommandGroup {

    DriveTrainTalonSRX m_driveTrain;

    public MotionMagicDriveTest(DriveTrainTalonSRX driveTrain) {

        m_driveTrain = driveTrain;

        addCommands(new DriveMMTest(m_driveTrain, 24));

    }
    
}
