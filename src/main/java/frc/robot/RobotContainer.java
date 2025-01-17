// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TeleopDrive;
import frc.robot.subSystems.*;


public class RobotContainer {
  private XboxController driverController;
  private Chassis chassis;

  public RobotContainer() {
    driverController = new XboxController(Constants.DRIVER_PORT);
    chassis = new Chassis();
    chassis.setDefaultCommand(new TeleopDrive(chassis, () -> driverController.getRawAxis(Constants.SPEED), () -> driverController.getRawAxis(Constants.ROTATION_SPEED)));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
