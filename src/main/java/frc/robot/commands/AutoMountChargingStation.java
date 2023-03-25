// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class AutoMountChargingStation extends CommandBase {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // PURPOSE: This command drives the robot back to the charging station after having              //
  //          driven completely over it from the starting point.  It should be used before         //
  //          running Stabilize that will use PID control to autobalance on the charging station.  //
  //          This should at least drive the robot to a point that it has pushed down the          //
  //          charging station, and possibly even drive a bit onto the charging station.           //
  //          But, be careful to not go so far as to cause an erratic oscillation.                 //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private Drive_Train _driveTrain;
  private double _startingPosition;

  public AutoMountChargingStation(Drive_Train driveTrain) {
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
    // Drive a little slowly to avoid tipping the bot, but doesn't need to be as slow as
    //  AutoDriveOntoChargingStation (40% power => tune this number)
    _driveTrain.drive(0, -0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Travel from the outside edge of the charging station back to the charging station,
    //  with the bot pushing down the outside of the charging station
    //  (~0.45 meters [? roughly half the chassis length?] => tune this number to finish
    //   where the bot is pushing down the charging station)
    double currentPosition = _driveTrain.getPosition();
    if (Math.abs(currentPosition - _startingPosition) >= 0.45) return true;

    return false;
  }
}
