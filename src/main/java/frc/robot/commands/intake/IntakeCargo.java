package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {

    XboxController m_controller;

    Intake m_intake;

    public IntakeCargo(Intake intake, XboxController controller) {
        
        m_intake = intake;
        m_controller = controller;

        addRequirements(m_intake);

    }

    @Override
    public void initialize() {
        m_intake.collectBalls();
        m_intake.moveIntakeDown();
        m_controller.setRumble(RumbleType.kLeftRumble, 0.2);
        m_controller.setRumble(RumbleType.kRightRumble, 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_intake.switchIntake();
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
    }
    
}
