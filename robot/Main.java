// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 * This is because all initialization should happen in your Robot class. The RIO may not be capable
 * of initializing variables and classes statically here.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    /*
     * RobotBase.startRobot is an interesting and strangely confusing function
     * The function takes in another function as a parameter, but we want to give it
     * the Robot constructor. Java does not allocate class instances onto the stack, so
     * we cannot give it the constructor directly (Robot::Robot) BUT we can pass in the
     * 'new' operator as a function. All operators are technically functions! The :: syntax
     * allows us to get a function 'pointer' and pass it to the function, or even
     * store it into a variable if we wanted to.
     */
    RobotBase.startRobot(Robot::new);
  }
}
