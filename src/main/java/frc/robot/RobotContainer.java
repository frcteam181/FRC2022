package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.shuffleboard.TestDriveTab;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

import java.util.List;

@SuppressWarnings("unused")
public class RobotContainer {

  // Subsystems
  private DriveTrain m_driveTrain;
  private Intake m_intake;
  private Conveyor m_conveyor;
  private Climber m_climber;
  private Shooter m_shooter;

  // Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;

  // Shuffleboard Tabs
  private TestDriveTab m_testDriveTab;
  private CompetitionTab m_competitionTab;
  
  public RobotContainer() {

    // Subsystems
    m_driveTrain = new DriveTrain();
    m_intake = new Intake();
    m_conveyor = new Conveyor();
    m_climber = new Climber();
    m_shooter = new Shooter();

    // Controllers
    m_driverController = new DriverController(m_driveTrain);
    m_operatorController = new OperatorController(m_intake, m_shooter, m_climber, m_conveyor);

    // Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_shooter);
    m_testDriveTab = new TestDriveTab(m_driveTrain);

  }

  public Command getAutonomousCommand() {
    
    //return m_competitionTab.getAuto();
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared).setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_driveTrain::getPose, new RamseteController(kRamseteB, kRamseteZeta), new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, m_driveTrain::getWheelSpeeds, new PIDController(kPDriveVel, 0, 0),
            new PIDController(kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_driveTrain::tankDriveVolts,
            m_driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
  }
}
