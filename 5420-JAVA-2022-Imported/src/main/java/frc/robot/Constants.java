package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public final class Constants {
    public static final class DriveTrainConstants {
        public static final double kMaxSpeed = 2; // Meters per second
        public static final double kMaxAngularSpeed = Math.PI * 0.75 ; // 1/2 rotation per second

        public static final double driveSpeedP = 0.18;
        public static final double driveSpeedI = 0;
        public static final double driveSpeedD = 0;

        public static final double driveP = 0.1;
        public static final double driveI = 0;
        public static final double driveD = 0;

        public static final double turnP = 0.3;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final double limeP = 0.08;
        public static final double limeI = 0;
        public static final double limeD = 0;

        public static final double driveMaxOutput = 0.8;
        public static final double turnMaxOutput = 0.5;

        public static final double driveEncoderMultiplier = (Math.PI * 0.102) / 14150;
        public static final double turnEncoderMultiplier = (2 * Math.PI) / 360;

        // CAN id
        public static final int frontLeftDrive = 6;
        public static final int frontLeftTurn = 5;
        public static final int frontLeftEncoder = 59;

        public static final int frontRightDrive = 4;
        public static final int frontRightTurn = 3;
        public static final int frontRightEncoder = 61;

        public static final int backLeftDrive = 8;
        public static final int backLeftTurn = 7;
        public static final int backLeftEncoder = 60;

        public static final int backRightDrive = 2;
        public static final int backRightTurn = 1;
        public static final int backRightEncoder = 62;

        public static final int pigeon = 24;

        // Pixy
        public static final Link pixyLink = new SPILink();
        public static final SPI.Port pixyLinkPort = SPI.Port.kOnboardCS0;
        
        public static final double pixyTargetArea = 10000;
    }

    public static final class ShooterConstants {
        public static final int shooterMotor1 = 10;
        public static final int shooterMotor2 = 9;
        public static final int feedMotor = 55;
    }

    public static final class ControllerConstants {
        // Logitech controllers
        public static final int JOYSTICK_RIGHT_X_AXIS = 4;
        public static final int JOYSTICK_RIGHT_Y_AXIS = 5;
        public static final int JOYSTICK_LEFT_X_AXIS = 0;
        public static final int JOYSTICK_LEFT_Y_AXIS = 1;

        public static final int Red_Button_ID = 2;
        public static final int Green_Button_ID = 1;
        public static final int Yellow_Button_ID = 4;
        public static final int Blue_Button_ID = 3;

        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Right_Trigger = 3;
        public static final int Left_Trigger = 2;

        public static final int Joystick_Left_Button = 9;
        public static final int Joystick_Right_Button = 10;

        // Range for inputs that we will consider to be no input
        public static double NoInputTolerance = 0.25;
    }

    public static final class xJoystickConstants {
        // Xtreme 3d joystick map
        public static final int axis_x = 0;
        public static final int axis_y = 1;
        public static final int axis_rot = 2;
        public static final int axis_throttle = 3;
        public static final int Button_Triangle = 2;
        public static final int Button_X = 3;
        public static final int Button_Square = 4;
        public static final int Button_Select = 11;
        public static final int Button_Start = 12;
        public static final int Button_PlayStation = 13;
        public static final int Button_L3 = 9;
        public static final int Button_R3 = 10;
        public static final int Button_HatSwitchUp = 5;
        public static final int Button_HatSwitchRight = 6;
        public static final int Button_HatSwitchDown = 7;
        public static final int Button_HatSwitchLeft = 8;
        public static final int Button_Trigger = 1;

        public static final double rotationTolerance = 0.3;
    }

    public static final class ThrustMasterJoystick{
        public static final int Axis_X = 0;
        public static final int Axis_Y = 1;
        public static final int Axis_Rot = 2;
        public static final int Axis_Throttle = 3;
        public static final int Button_Trigger = 1;
        public static final int Button_Thumb_Down = 2;
        public static final int Button_Thumb_Left = 3;
        public static final int Button_Thumb_Right = 4;
        public static final int Button_Left_Left = 5;
        public static final int Button_Left_Middle = 6;
        public static final int Button_Left_Right = 7;
        public static final int Button_Right_Right = 11;
        public static final int Button_Right_Middle = 12;
        public static final int Button_Right_Left = 13;
    }
}