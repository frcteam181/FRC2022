package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
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

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {



  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
