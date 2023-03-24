// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive_Train _drive_Train = new Drive_Train();
  private final IntakePivot _intakePivot = new IntakePivot();
  private final Booty_Intake _bootyIntake = new Booty_Intake();
  private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

  private SendableChooser<String> _chooser = new SendableChooser<String>();
  private String _autoSelected;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    _chooser.setDefaultOption("Using constructed class", "Using constructed class");
    _chooser.addOption("Using class factory (inline)", "Using class factory (inline)");
    _chooser.addOption("Using PID stabilize", "Using PID stabilize");

    SmartDashboard.putData("Auto choices", _chooser);
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    _autoSelected = _chooser.getSelected();

    if (_autoSelected == "Using constructed class") {
      return new Auto18PointSequence(_intakePivot, _bootyIntake, _drive_Train);
    } else if (_autoSelected == "Using class factory (inline)") {
      return new AutoPivotUp(_intakePivot)
        .andThen(new AutoIntakeOut(_bootyIntake))
        .andThen(new AutoDriveOverChargingStation(_drive_Train))
        .andThen(new AutoDriveBackToChargingStation(_drive_Train));
    } else if (_autoSelected == "Using PID stabilize") {
      return new AutoPivotUp(_intakePivot)
        .andThen(new AutoIntakeOut(_bootyIntake))
        .andThen(new AutoDriveOverChargingStation(_drive_Train))
        .andThen(new AutoDriveBackToChargingStation(_drive_Train))
        .andThen(new AutoStabilize(_drive_Train, _gyro));
    }

    return null;  // do nothing
  }
}
