// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.LQRFlyWheel;
import frc.robot.subsystems.PIDFlyWheel;
import frc.robot.subsystems.SimpleFlyWheel;

public class RobotContainer {
  // Gamepads
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_sysIDcontroller = new CommandXboxController(1);

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

    // While the B button is held down, then run the spin command at 2 volts
    m_controller.a().whileTrue(m_simpleFlyWheel.spinCommand(2));

    // While the B button is held down, then run the spin command at 2 volts
    m_controller.b().whileTrue(m_simpleFlyWheel.spinCommand(4));

    // While the X button is held down, then run the spin command at 2 volts
    m_controller.x().whileTrue(m_simpleFlyWheel.spinCommand(6));

    // While the Y button is held down, then run the spin command at 2 volts
    m_controller.y().whileTrue(m_simpleFlyWheel.spinCommand(8));


    //
    // PID FlyWheel
    //

    // While the start button is held down, then spin the fly wheel at 0.1 (10%) output
    // Useful to checkout the flywheel
    m_controller.start().whileTrue(m_pidFlywheel.setOpenLoopCommand(2));

    // While the A button is held down, then spin the fly wheel at 500 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(500));

    // While the B button is held down, then spin the fly wheel at 1000 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(1000));

    // While the X button is held down, then spin the fly wheel at 1500 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(1500));

    // While the Y button is held down, then spin the fly wheel at 2000 RPM
    m_controller.a().whileTrue(m_pidFlywheel.setClosedLoopPIDCommand(2000));

    //
    // LQR FlyWheel
    //

    // While the start button is held down, then spin the fly wheel at 0.1 (10%) output
    // Useful to checkout the flywheel
    m_controller.start().whileTrue(m_lqrFlywheel.setOpenLoopCommand(2));

    // While the A button is held down, then spin the fly wheel at 500 RPM
    m_controller.a().whileTrue(m_lqrFlywheel.setClosedLoopLQRCommand(500));

    // While the B button is held down, then spin the fly wheel at 1000 RPM
    m_controller.b().whileTrue(m_lqrFlywheel.setClosedLoopLQRCommand(1000));

    // While the X button is held down, then spin the fly wheel at 1500 RPM
    m_controller.x().whileTrue(m_lqrFlywheel.setClosedLoopLQRCommand(1500));

    // While the Y button is held down, then spin the fly wheel at 2000 RPM
    m_controller.y().whileTrue(m_lqrFlywheel.setClosedLoopLQRCommand(2000));

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    m_sysIDcontroller.a().whileTrue(m_lqrFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_sysIDcontroller.b().whileTrue(m_lqrFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_sysIDcontroller.x().whileTrue(m_lqrFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_sysIDcontroller.y().whileTrue(m_lqrFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
