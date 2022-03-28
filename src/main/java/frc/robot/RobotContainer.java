package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.drive_train.DriveMM;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.commands.drive_train.TurnToAngle;
import frc.robot.commands.drive_train.TurnToAngleTest;
import frc.robot.commands.drive_train.UpdateDriveLimiters;
import frc.robot.subsystems.DriveTrain;


public class RobotContainer {

  //Subsystems
  DriveTrain m_driveTrain;

  //OI
  Joystick m_driverController;

  private SendableChooser<Command> m_autoChooser;
  
  public RobotContainer() {

    //Subsystems
    m_driveTrain = new DriveTrain();

    //Default Commands
    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_driverController));

    // Builds the Shuffleboard Tabs
    buildShuffleboard();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {



  }

  private void buildShuffleboard() {

    buildDriverTestTab();

  }

  private void buildDriverTestTab() {

    ShuffleboardTab m_driveMMTab = Shuffleboard.getTab("Drive Testing");

    // Config values on row 1
    m_driveMMTab.add("kF", 0.0 )              .withPosition(0, 0).getEntry();
    m_driveMMTab.add("kP", 0.0 )              .withPosition(1, 0).getEntry();
    m_driveMMTab.add("kI", 0.0 )                .withPosition(2, 0).getEntry();
    m_driveMMTab.add("kD", 0.0 )                .withPosition(3, 0).getEntry();
    m_driveMMTab.add("Tgt. Inches", 0.0)        .withPosition(4, 0).getEntry();
    m_driveMMTab.add("Tgt. Degrees", 0.0)       .withPosition(5, 0).getEntry();
    m_driveMMTab.add("Finish Iterations", 5 ) .withPosition(6, 0).getEntry();

    // Result Values on row 2
    m_driveMMTab.add("Tgt. Ticks", 0)                                          .withPosition(0, 1);
    m_driveMMTab.addNumber("Left Encoder", m_driveTrain::getLeftEncoderPosition)  .withPosition(1, 1);
    m_driveMMTab.addNumber("Right Encoder", m_driveTrain::getRightEncoderPosition).withPosition(2, 1);
    m_driveMMTab.addNumber("Gyro Read", m_driveTrain::getRawAngle)             .withPosition(3, 1);
    m_driveMMTab.add("Run Time", 0)                                            .withPosition(4, 1);
    m_driveMMTab.addNumber("Left SP", m_driveTrain::getLeftSetPoint).withPosition(5, 1).withSize(1, 1);
    m_driveMMTab.addNumber("Right SP", m_driveTrain::getRightSetPoint).withPosition(6, 1).withSize(1, 1);

    // Drive limiters on row 3
    m_driveMMTab.add("Forward Limiter", 2.5).withPosition(0, 2);
    m_driveMMTab.add("Rotation Limiter", 2.5).withPosition(1, 2);
    m_driveMMTab.add("Drive Max", .7).withPosition(2, 2);
    m_driveMMTab.add("Update Limits", new UpdateDriveLimiters(m_driveTrain)).withPosition(3, 2).withSize(2, 1);

    // Drive commands on row 4
    m_driveMMTab.add("Drive MM 100", new DriveMM(m_driveTrain, 100))        .withPosition(0, 3).withSize(2, 1);
    m_driveMMTab.add("Drive MM -100", new DriveMM(m_driveTrain, -100))      .withPosition(2, 3).withSize(2, 1);
    m_driveMMTab.add("Drive MM Test", new DriveMMTest(m_driveTrain, 0))     .withPosition(4, 3).withSize(2, 1);

    // Turn commands on row 5
    m_driveMMTab.add("Turn MM 90", new TurnToAngle(m_driveTrain, 90))          .withPosition(0, 4).withSize(2, 1);
    m_driveMMTab.add("Turn MM -90", new TurnToAngle(m_driveTrain, -90))        .withPosition(2, 4).withSize(2, 1);
    m_driveMMTab.add("Turn MM 180", new TurnToAngle(m_driveTrain, 180))     .withPosition(4, 4).withSize(2, 1);
    m_driveMMTab.add("Turn MM Test", new TurnToAngleTest(m_driveTrain, 0))     .withPosition(6, 4).withSize(2, 1);

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
