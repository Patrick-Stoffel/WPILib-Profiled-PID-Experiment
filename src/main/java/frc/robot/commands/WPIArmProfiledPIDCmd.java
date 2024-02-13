// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class WPIArmProfiledPIDCmd extends Command {
  private final ArmSubsystem armSubsystem;
  private final ProfiledPIDController profiledPIDController;
//  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.12, 0);

  public WPIArmProfiledPIDCmd(ArmSubsystem armSubsystem, double setPoint) {
    this.armSubsystem = armSubsystem;
//    this.setPoint = setPoint;
    this.profiledPIDController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    profiledPIDController.setGoal(setPoint);                                                                // In Rotations/Sec 
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      profiledPIDController.reset(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double speed = profiledPIDController.calculate(armSubsystem.getArmPosition());   // + armFeedforward.calculate.....
      armSubsystem.setArmSpeed(speed);
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
