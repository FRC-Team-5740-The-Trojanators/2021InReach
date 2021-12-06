// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int frontLeftCAN = 4;
    public static int frontRightCAN = 2;
    public static int backLeftCAN = 3;
    public static int backRightCAN = 5;

    public static double PTurn = 11111111111111.1;
    public static double ITurn = 11111111111111.1;
    public static double DTurn = 11111111111111.1;
    public static double turnEpsilon = 11111111111111.1;

    public static double PDrive = 1111111111111.1;
    public static double IDrive = 1111111111111.1;
    public static double DDrive = 1111111111111.1;

    public static int kDriverPort = 1;

    public static int kRightStickX = 4;
    public static int kLeftStickY = 5;

    public static int[] kLeftEncoderPorts = new int[]{0,1};
    public static int[] kRightEncoderPorts = new int[]{0,1};

    public static boolean kLeftEncoderReversed = true;
    public static boolean kRightEncoderReversed = true;

    public static double kEncoderDistancePerPulse = 100000000000000000.0;


    



}
