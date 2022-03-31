package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase{

    Intake m_intake;

    public MoveIntake(Intake intake) {

        m_intake = intake;

        addRequirements(intake);

    }

    @Override
    public void execute() {
        
        m_intake.move_piston();

    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
    
}