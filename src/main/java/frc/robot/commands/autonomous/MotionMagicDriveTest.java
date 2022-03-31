package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.subsystems.DriveTrain;

public class MotionMagicDriveTest extends SequentialCommandGroup {

    DriveTrain m_driveTrain;

    public MotionMagicDriveTest(DriveTrain driveTrain) {

        m_driveTrain = driveTrain;

        addCommands(new DriveMMTest(m_driveTrain, 24));

    }
    
}
