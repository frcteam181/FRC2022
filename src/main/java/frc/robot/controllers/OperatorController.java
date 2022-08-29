package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.default_commands.ClimberDefaultCommand;
import frc.robot.commands.default_commands.ConveyorDefaultCommand;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.IntakeCargo;
import frc.robot.commands.shooter.NearShot;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;

public class OperatorController {

    private XboxController m_conveyorController, m_climberController;

    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs,
                            m_c_a, m_c_b, m_c_x, m_c_y, m_c_lb, m_c_rb, m_c_sl, m_c_st, m_c_ls, m_c_rs;

    private POVButton m_up, m_dw, m_l, m_r,
                        m_c_up, m_c_dw, m_c_l, m_c_r;

    private boolean m_isDefaultBinding;

    // Used Robot Subsystems //
    private Intake m_intake;
    private Climber m_climber;
    private Conveyor m_conveyor;
    private Shooter m_shooter;

    public OperatorController(Intake intake, Shooter shooter, Climber climber, Conveyor conveyor) {
        
        // Used Subsystems Instances //
        m_intake = intake;
        m_climber = climber;
        m_conveyor = conveyor;
        m_shooter = shooter;

        // Xbox Controllers
        m_conveyorController = new XboxController(kOPERATOR_CONTROLLER);
        m_climberController = new XboxController(kCLIMBER_CONTROLLER);

        // Xbox Controller Buttons
        m_st = new JoystickButton(m_conveyorController, XboxController.Button.kStart.value);
        m_sl = new JoystickButton(m_conveyorController, XboxController.Button.kBack.value);
        m_lb = new JoystickButton(m_conveyorController, XboxController.Button.kLeftBumper.value);
        m_rb = new JoystickButton(m_conveyorController, XboxController.Button.kRightBumper.value);
        m_ls = new JoystickButton(m_conveyorController, XboxController.Button.kLeftStick.value);
        m_rs = new JoystickButton(m_conveyorController, XboxController.Button.kRightStick.value);
        m_a = new JoystickButton(m_conveyorController, XboxController.Button.kA.value);
        m_b = new JoystickButton(m_conveyorController, XboxController.Button.kB.value);
        m_x = new JoystickButton(m_conveyorController, XboxController.Button.kX.value);
        m_y = new JoystickButton(m_conveyorController, XboxController.Button.kY.value);
        m_up = new POVButton(m_conveyorController, 0);
        m_r = new POVButton(m_conveyorController, 90);
        m_dw = new POVButton(m_conveyorController, 180);
        m_l = new POVButton(m_conveyorController, 270);

        m_c_st = new JoystickButton(m_conveyorController, XboxController.Button.kStart.value);
        m_c_sl = new JoystickButton(m_conveyorController, XboxController.Button.kBack.value);
        m_c_lb = new JoystickButton(m_conveyorController, XboxController.Button.kLeftBumper.value);
        m_c_rb = new JoystickButton(m_conveyorController, XboxController.Button.kRightBumper.value);
        m_c_ls = new JoystickButton(m_conveyorController, XboxController.Button.kLeftStick.value);
        m_c_rs = new JoystickButton(m_conveyorController, XboxController.Button.kRightStick.value);
        m_c_a = new JoystickButton(m_conveyorController, XboxController.Button.kA.value);
        m_c_b = new JoystickButton(m_conveyorController, XboxController.Button.kB.value);
        m_c_x = new JoystickButton(m_conveyorController, XboxController.Button.kX.value);
        m_c_y = new JoystickButton(m_conveyorController, XboxController.Button.kY.value);
        m_c_up = new POVButton(m_conveyorController, 0);
        m_c_r = new POVButton(m_conveyorController, 90);
        m_c_dw = new POVButton(m_conveyorController, 180);
        m_c_l = new POVButton(m_conveyorController, 270);

        // Default Command
        m_conveyor.setDefaultCommand(new ConveyorDefaultCommand(m_conveyor, m_conveyorController));
        m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, m_climberController));

        bindButtons();

    }

    public void bindButtons() {
        m_rb.whileHeld(new IntakeCargo(m_intake, m_conveyorController));
        m_lb.whileHeld(new EjectCargo(m_intake, m_conveyorController));
        m_y.whenPressed(new NearShot(m_shooter));
        m_x.whenPressed(new StopShooter(m_shooter));
        m_a.whenPressed(m_intake::switchIntake);
        m_b.whenPressed(m_climber::tiltClimber);
    }

    public void switchBindings() {
        m_isDefaultBinding = !m_isDefaultBinding;
    }
    
}
