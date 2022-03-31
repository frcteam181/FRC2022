package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.TwoBallsLeft;
import frc.robot.commands.climber.AutoClimbUp;
import frc.robot.commands.climber.ExtendClimberBoth;
import frc.robot.commands.climber.RetractClimberBoth;
import frc.robot.commands.climber.TiltClimberBackward;
import frc.robot.commands.climber.TiltClimberForward;
import frc.robot.commands.default_commands.ClimberDefaultCommand;
import frc.robot.commands.default_commands.ConveyorDefaultCommand;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.drive_train.DriveMM;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.commands.drive_train.InvertDrive;
import frc.robot.commands.drive_train.TurnToAngle;
import frc.robot.commands.drive_train.TurnToAngleTest;
import frc.robot.commands.drive_train.UpdateDriveLimiters;
import frc.robot.commands.intake.MoveIntake;
import frc.robot.commands.intake.moveDownAndIntake;
import frc.robot.commands.intake.moveDownAndOutake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;


public class RobotContainer {

  // Subsystems
  DriveTrain m_driveTrain;
  Intake m_intake;
  Conveyor m_conveyor;
  Climber m_climber;

  // Controllers
  Joystick m_driverController;
  XboxController m_operatorController, m_climberController;

  // Buttons
  JoystickButton m_d_t,
                  m_o_a, m_o_b, m_o_x, m_o_y, m_o_lb, m_o_rb, m_o_sl, m_o_st, m_o_ls, m_o_rs,
                    m_c_a, m_c_b, m_c_x, m_c_y, m_c_lb, m_c_rb, m_c_sl, m_c_st, m_c_ls, m_c_rs;
  POVButton m_o_up, m_o_dw, m_o_l, m_o_r,
            m_c_up, m_c_dw, m_c_l, m_c_r;
  

  private SendableChooser<Command> m_autoChooser;
  
  public RobotContainer() {

    // Controllers
    m_driverController = new Joystick(kDRIVER_CONTROLLER);
    m_operatorController = new XboxController(kOPERATOR_CONTROLLER);
    m_climberController = new XboxController(kCLIMBER_CONTROLLER);

    // Driver Buttons
    m_d_t = new JoystickButton(m_driverController, ButtonType.kTrigger.value);

    // Operator Buttons
    m_o_st = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);
    m_o_sl = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);
    m_o_lb = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
    m_o_rb = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
    m_o_ls = new JoystickButton(m_operatorController, XboxController.Button.kLeftStick.value);
    m_o_rs = new JoystickButton(m_operatorController, XboxController.Button.kRightStick.value);
    m_o_a = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    m_o_b = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    m_o_x = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
    m_o_y = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
    m_o_up = new POVButton(m_operatorController, 0);
    m_o_r = new POVButton(m_operatorController, 90);
    m_o_dw = new POVButton(m_operatorController, 180);
    m_o_l = new POVButton(m_operatorController, 270);

    // Climber Buttons
    m_c_st = new JoystickButton(m_climberController, XboxController.Button.kStart.value);
    m_c_sl = new JoystickButton(m_climberController, XboxController.Button.kBack.value);
    m_c_lb = new JoystickButton(m_climberController, XboxController.Button.kLeftBumper.value);
    m_c_rb = new JoystickButton(m_climberController, XboxController.Button.kRightBumper.value);
    m_c_ls = new JoystickButton(m_climberController, XboxController.Button.kLeftStick.value);
    m_c_rs = new JoystickButton(m_climberController, XboxController.Button.kRightStick.value);
    m_c_a = new JoystickButton(m_climberController, XboxController.Button.kA.value);
    m_c_b = new JoystickButton(m_climberController, XboxController.Button.kB.value);
    m_c_x = new JoystickButton(m_climberController, XboxController.Button.kX.value);
    m_c_y = new JoystickButton(m_climberController, XboxController.Button.kY.value);
    m_c_up = new POVButton(m_climberController, 0);
    m_c_r = new POVButton(m_climberController, 90);
    m_c_dw = new POVButton(m_climberController, 180);
    m_c_l = new POVButton(m_climberController, 270);

    // Subsystems
    m_driveTrain = new DriveTrain();
    m_intake = new Intake();
    m_conveyor = new Conveyor();
    m_climber = new Climber();

    //Default Commands
    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_driverController));
    m_conveyor.setDefaultCommand(new ConveyorDefaultCommand(m_conveyor, m_operatorController));
    m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, m_climberController));

    // Builds the Shuffleboard Tabs
    buildShuffleboard();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Driver Bindings
    m_d_t.whenPressed(new InvertDrive(m_driveTrain));

    // Operator Bindings
    m_o_lb.whileHeld(new moveDownAndOutake(m_intake));
    m_o_rb.whileHeld(new moveDownAndIntake(m_intake));
    m_o_a.whenPressed(new MoveIntake(m_intake));

    // Operator Climb Bindings
    m_c_st.whileHeld(new ExtendClimberBoth(m_climber));
    m_c_sl.whileHeld(new RetractClimberBoth(m_climber));
    m_c_lb.whileHeld(new TiltClimberForward(m_climber));
    m_c_rb.whileHeld(new TiltClimberBackward(m_climber));
    m_c_a.whenPressed(new MoveIntake(m_intake));
    m_c_b.whenPressed(new AutoClimbUp(m_climber));
    
  }

  private void buildShuffleboard() {

    buildCompetitionTab();
    buildDriverTestTab();

  }

  private void buildCompetitionTab() {

    ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

    // Auto Chooser
    m_autoChooser = new SendableChooser<Command>();

    // Sets Default Auto
    //m_autoChooser.setDefaultOption("Taxi Only", new TaxiOnly(m_driveTrain));
    
    // Add autos here:
    m_autoChooser.addOption("Hangar : Two Ball", new TwoBallsLeft(m_driveTrain, m_intake, m_conveyor));

    // 
    m_competitionTab.add("Choose Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 3).withSize(2, 1);
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
    m_driveMMTab.add("Forward Limiter", 0.5).withPosition(0, 2);
    m_driveMMTab.add("Rotation Limiter", 0.5).withPosition(1, 2);
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

    // Current Drive PID on column 8
    m_driveMMTab.add("Drive Kp", m_driveTrain.getPIDValue("Drive", "Kp"))          .withPosition(8, 0).withSize(1, 1);
    m_driveMMTab.add("Drive Ki", m_driveTrain.getPIDValue("Drive", "Ki"))        .withPosition(8, 1).withSize(1, 1);
    m_driveMMTab.add("Drive Kd", m_driveTrain.getPIDValue("Drive", "Kd"))     .withPosition(8, 2).withSize(1, 1);
    m_driveMMTab.add("Drive Kf", m_driveTrain.getPIDValue("Drive", "Kf"))     .withPosition(8, 3).withSize(1, 1);

    // Current Drive PID on column 9
    m_driveMMTab.add("Turn Kp", m_driveTrain.getPIDValue("Turn", "Kp"))          .withPosition(9, 0).withSize(1, 1);
    m_driveMMTab.add("Turn Ki", m_driveTrain.getPIDValue("Turn", "Ki"))        .withPosition(9, 1).withSize(1, 1);
    m_driveMMTab.add("Turn Kd", m_driveTrain.getPIDValue("Turn", "Kd"))     .withPosition(9, 2).withSize(1, 1);
    m_driveMMTab.add("Turn Kf", m_driveTrain.getPIDValue("Turn", "Kf"))     .withPosition(9, 3).withSize(1, 1);

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
