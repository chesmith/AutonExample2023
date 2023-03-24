// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class AutoDriveOverChargingStation extends CommandBase {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // PURPOSE: This command drives the robot completely over the charging station from the starting //
  //          point in order to get the "mobility" points.                                         //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private Drive_Train _driveTrain;
  private double _startingPosition;

  public AutoDriveOverChargingStation(Drive_Train driveTrain) {
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
    _driveTrain.drive(0, 0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Travel the distance from center starting position (bumpers against the grid)
    //  to the other side of the charging station (~4 meters => tune this value)
    // Keep in mind the bumpers need to fully exit the community, so this distance
    //   is effectively "travel distance over the charging station" + "robot length with bumpers"
    double currentPosition = _driveTrain.getPosition();
    if (Math.abs(currentPosition - _startingPosition) >= 4.0) return true;

    return false;
  }
}
