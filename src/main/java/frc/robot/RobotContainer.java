// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SimpleFlyWheel;

public class RobotContainer {
  // Gamepads
  private final CommandXboxController m_controller = new CommandXboxController(0);

  // Subsystems
  private final SimpleFlyWheel m_simpleFlyWheel = new SimpleFlyWheel();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Setup button to command bindings

    // While the B button is held down, then run the spin command
    m_controller.a().whileTrue(m_simpleFlyWheel.spinCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
