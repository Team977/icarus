/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class tankDrive extends Command {

  private double[] error = {0,0,0,0};
  private double[] sumError = {0,0,0,0};
  private double[] diffError = {0,0,0,0};
  private double[] prevError = {0,0,0,0};
  private double[] turnPower = {0,0,0,0};

  private final double kP = 0.5;
  private final double kI = 0.001;
  private final double kD = 0.1;

  public tankDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.robotDrivetrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    for(int i=0; i<4; i++){
      
      error[i] = 0 - Robot.robotDrivetrain.getEncoderDegrees()[i];
      sumError[i] += error[i];
      diffError[i] = error[i] - prevError[i];
      prevError[i] = error[i];

      turnPower[i] = error[i]*kP + sumError[i]*kI + diffError[i]*kD;
    }

    //differential drive calcs here



  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
