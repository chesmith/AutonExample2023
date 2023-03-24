// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class AutoStabilize extends CommandBase {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // PURPOSE: This command uses PID control to automatically balance on the charging station.      //
  //          It assumes the bot is already somewhat on the charging station and sitting at an     //
  //          angle.  Recommended to have the bot far enough to have pushed down the edge before   //
  //          starting this command.                                                               //
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  private Drive_Train _driveTrain;
  private AHRS _gyro;

  private final double _Kp = 0.4;
  private final double _Ki = 0.15;
  private final double _Kd = 0;
  private PIDController _pid = new PIDController(_Kp, _Ki, _Kd);

  public AutoStabilize(Drive_Train driveTrain, AHRS gyro) {
    _driveTrain = driveTrain;
    _gyro = gyro;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = -_gyro.getPitch();
    SmartDashboard.putNumber("Error", error);
    double pidOut = _pid.calculate(error, 0);
    SmartDashboard.putNumber("PID Out", pidOut);

    double drivePower = pidOut / 20;
    SmartDashboard.putNumber("Drive Power", drivePower);
    if (Math.abs(drivePower) > 0.5) {
      drivePower = Math.copySign(0.5, drivePower);
    }
    if (Math.abs(drivePower) < 0.20 && (Math.abs(pidOut)) < 6) {
      drivePower = Math.copySign(0.20, drivePower);
    }
    if (Math.abs(error) > 2) {
      _driveTrain.drive(0, drivePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}