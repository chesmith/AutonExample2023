// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePivot;

public class AutoPivotUp extends CommandBase {
  private final IntakePivot _intakePivot;

  public AutoPivotUp(IntakePivot intakePivot) {
    _intakePivot = intakePivot;
    addRequirements(_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intakePivot.pivotUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // motor and encoder are installed such that up = negative, down = positive
    // ideally, we would have adjusted configuration somehow to return up = positive, but whatever... this will still work
    return (_intakePivot.getPosition() < -20);
  }
}
