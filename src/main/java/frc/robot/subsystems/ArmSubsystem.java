// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax armNeo;
  RelativeEncoder armEncoder;

  public ArmSubsystem() {
    armNeo = new CANSparkMax(20, MotorType.kBrushless);
    armNeo.restoreFactoryDefaults();

    armEncoder = armNeo.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42); // Internal relative encoder
   //  armEncoder = armNeo.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);   Rev thru-bore encoder
  
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmSpeed(double speed) {
      armNeo.set(speed);
  }

  public double getArmPosition() {
      return armEncoder.getPosition();
  }

}
