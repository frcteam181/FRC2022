package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainDefaultCommand extends CommandBase{

    private final DriveTrain m_driveTrain;
    private final Joystick m_driverController;

    public DriveTrainDefaultCommand(DriveTrain driveTrain, Joystick driverController) {
        m_driveTrain = driveTrain;
        m_driverController = driverController;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {

        double m_forwardValue, m_turnValue;
        m_forwardValue = -m_driverController.getY();
        m_turnValue = m_driverController.getZ();
        m_driveTrain.teleopDrive(m_forwardValue, m_turnValue, false);

    }
    
}
