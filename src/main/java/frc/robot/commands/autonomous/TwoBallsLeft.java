package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.conveyor.SpitCargo;
import frc.robot.commands.conveyor.horizontalBelt;
import frc.robot.commands.drive_train.DriveMM;
import frc.robot.commands.drive_train.TurnToAngle;
import frc.robot.commands.intake.moveDownAndIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class TwoBallsLeft extends SequentialCommandGroup{

    DriveTrain m_driveTrain;
    Intake m_intake;
    Conveyor m_conveyor;

    public TwoBallsLeft(DriveTrain driveTrain, Intake intake, Conveyor conveyor) {
        
        m_driveTrain = driveTrain;
        m_intake = intake;
        m_conveyor = conveyor;

        addCommands(new ParallelCommandGroup(new DriveMM(m_driveTrain, -110).withTimeout(3), new WaitCommand(1.5).andThen(new moveDownAndIntake(m_intake).withTimeout(1.5)), new WaitCommand(1.5).andThen(new horizontalBelt(m_conveyor).withTimeout(1.5))), 
                    new DriveMM(m_driveTrain, 110).withTimeout(3), 
                    new ParallelCommandGroup(new SpitCargo(m_conveyor).withTimeout(1.5), new WaitCommand(1.5).andThen(new DriveMM(m_driveTrain, -36).withTimeout(3))),
                    new TurnToAngle(m_driveTrain, 90).withTimeout(3),
                    new ParallelCommandGroup(new DriveMM(m_driveTrain, -110).withTimeout(5), new WaitCommand(1.5).andThen(new moveDownAndIntake(m_intake).withTimeout(3.5)), new WaitCommand(1.5).andThen(new horizontalBelt(m_conveyor).withTimeout(3.5))),
                    new DriveMM(m_driveTrain, 110).withTimeout(5),
                    new TurnToAngle(m_driveTrain, -90).withTimeout(3),
                    new DriveMM(m_driveTrain, 36).withTimeout(3),
                    new SpitCargo(m_conveyor).withTimeout(3)
                    );

    }
    
}
