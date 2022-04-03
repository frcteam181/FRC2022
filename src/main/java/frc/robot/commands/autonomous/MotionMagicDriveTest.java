package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.subsystems.DriveTrain;

public class MotionMagicDriveTest extends SequentialCommandGroup {

    DriveTrain m_driveTrain;
    Double m_targetInches;

    public MotionMagicDriveTest(DriveTrain driveTrain) {

        m_driveTrain = driveTrain;
        
        NetworkTable m_driveTestTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");
        m_targetInches = Double.valueOf(m_driveTestTable.getEntry("Tgt. Inches").toString()); // Trying to convert NetworkTableEntry -> String -> Double. If it doesn't work delete the conversions

        addCommands(new DriveMMTest(m_driveTrain, m_targetInches));

    }
    
}
