// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LQRFlyWheel;
import frc.robot.subsystems.PIDFlyWheel;
import frc.robot.subsystems.SimpleFlyWheel;

public class RobotContainer {
  // Gamepads
  private final CommandXboxController m_controller = new CommandXboxController(0);

  // Subsystems
  // TODO for this code to run, only one of the flywheel subsystems and commands needs to be uncommented out.
  private final SimpleFlyWheel m_simpleFlyWheel = new SimpleFlyWheel();
  private final PIDFlyWheel m_pidFlywheel = new PIDFlyWheel();
  private final LQRFlyWheel m_lqrFlywheel = new LQRFlyWheel();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Setup button to command bindings

    //
    // Simple FlyWheel
    //

    // While the B button is held down, then run the spin command
    m_controller.a().whileTrue(m_simpleFlyWheel.spinCommand());


    //
    // PID FlyWheel
    //

    // While the start button is held down, then spin the fly wheel at 0.1 (10%) output
    // Useful to checkout the flywheel
    m_controller.a().whileTrue(m_pidFlywheel.setOpenLoopCommand(0.1));

    // While the A button is held down, then spin the fly wheel at 500 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(500));

    // While the B button is held down, then spin the fly wheel at 1000 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(1000));

    // While the X button is held down, then spin the fly wheel at 1500 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(1500));

    // While the Y button is held down, then spin the fly wheel at 2000 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(2000));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void disabledInit() {
  }

  public void teleopInit() {
    m_lqrFlywheel.init();
  }

  public void autoInit() {
    m_lqrFlywheel.init();
  }
}
