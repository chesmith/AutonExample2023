// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class AutoDriveOntoChargingStation extends CommandBase {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // PURPOSE: This command drives the robot fully onto to the charging station after having        //
  //          driven completely over it from the starting point.                                   //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private Drive_Train _driveTrain;
  private double _startingPosition;

  public AutoDriveOntoChargingStation(Drive_Train driveTrain) {
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _startingPosition = _driveTrain.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive very slowly to avoid tipping the charging station (22% power => tune this number)
    _driveTrain.drive(0, -0.22);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Travel from the outside edge of the charging station back onto the charging station
    //  (~1.8 meters => tune this number to finish at the middle of the charging station)
    double currentPosition = _driveTrain.getPosition();
    if (Math.abs(currentPosition - _startingPosition) >= 1.8) return true;

    return false;
  }
}
