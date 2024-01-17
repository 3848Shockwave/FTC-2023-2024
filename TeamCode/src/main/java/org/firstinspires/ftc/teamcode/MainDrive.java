package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.locks.Lock;

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainDrive extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private final ElapsedTime runtime = new ElapsedTime();
        private DcMotor BackRight = null;
        private DcMotor BackLeft = null;
        private DcMotor FrontRight = null;
        private DcMotor FrontLeft = null;
        private DcMotor ArmMotor = null;
        private DcMotor Worm = null;
        Servo ArmServo;
        Servo ClawServoR;
        Servo ClawServoL;
        Servo LaunchServo;
        double increment = .2;
    boolean checkOne = false;
    boolean checkTwo = false;
    boolean clawR = false;
    boolean clawL = false;
    boolean checkThree = false;
    boolean Reload = false;
    long sleepTime = 50;
    double speedLimit = 1;
        @Override
        public void runOpMode() {


            FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
            BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
            FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            BackRight = hardwareMap.get(DcMotor.class, "BackRight");
            ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
            Worm = hardwareMap.get(DcMotor.class, "Worm");
            ArmServo = hardwareMap.get(Servo.class, "ArmServo");
            ClawServoR = hardwareMap.get(Servo.class, "ClawServoR");
            ClawServoL = hardwareMap.get(Servo.class, "ClawServoL");
            Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
            Worm.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;
                double max1;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = gamepad1.right_stick_y;
                double axial2   = gamepad1.left_stick_y;
                // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.right_trigger;
                double lateral1 =  gamepad1.left_trigger;
                double lateral2 =  -gamepad1.left_stick_x;
                double yaw     =  -gamepad1.right_stick_x;

                if(gamepad1.dpad_up){
                    yaw= -.14;
                }
                if(gamepad1.dpad_down){
                    yaw= .14;
                }
                double worm =   gamepad2.left_stick_y;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = ((axial2+axial) + ((-1*lateral)+lateral1+lateral2) + yaw);
                double rightFrontPower = ((axial2+axial) - ((-1*lateral)+lateral1+lateral2) - yaw);
                double leftBackPower   = ((axial2+axial) - ((-1*lateral)+lateral1+lateral2) + yaw);
                double rightBackPower  =((axial2+axial) + ((-1*lateral)+lateral1+lateral2) - yaw);
                double wormPower  = worm;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                max1 = Math.max(Math.abs(worm), Math.abs(rightFrontPower));
                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;

                }
                if (max1>1.0){
                    worm/=max1;
                }

                if (gamepad2.left_bumper){
                    ArmMotor.setPower(1);
                    //sleep(50);
                    ArmMotor.setPower(0);
                }
                if (gamepad2.right_bumper){
                    ArmMotor.setPower(-1);
                    //sleep(50);
                    ArmMotor.setPower(0);
                }
//            og worm code but milan no like    if (gamepad2.left_stick_y<0){
//                    Worm.setPower(-1);
//                    sleep(50);
//                    Worm.setPower(0);
//                }
//                if (gamepad2.left_stick_y>0){
//                    Worm.setPower(1);
//                    sleep(50);
//                    Worm.setPower(0);
//                }
                if (gamepad2.right_stick_y<0&&ArmServo.getPosition()+.002<=1){
                    ArmServo.setPosition(ArmServo.getPosition()+.002);

                    sleep(1);
                }
                if (gamepad2.right_stick_y>0&&ArmServo.getPosition()-.002>=.3){
                    ArmServo.setPosition(ArmServo.getPosition()-.002);
                    sleep(1);
                }
                if(gamepad2.x){
                    if (ClawServoL.getPosition()!=-1||ClawServoR.getPosition()!=1) {
                        ClawServoR.setPosition(1);
                        ClawServoL.setPosition(-1);

                    }
                        sleep(350);
                        if (ClawServoL.getPosition()==-1||ClawServoR.getPosition()==1) {
                            if (ArmServo.getPosition()!=1){
                            ArmServo.setPosition(1);
                        }

                    }

                }
                if(gamepad2.a) {
                    if (ClawServoL.getPosition() != .4) {
                        ClawServoL.setPosition(.4);
                    }
                    if (ClawServoR.getPosition() !=.5) {
                        ClawServoR.setPosition(.5);
                    }
                    if (ArmServo.getPosition() != .6) {
                        ArmServo.setPosition(.6);
                    }
                }
                if(gamepad2.b&& !clawR) {
                    if(ClawServoR.getPosition() == .5) ClawServoR.setPosition(1);
                    else ClawServoR.setPosition(.5);
                    clawR = true;
                } else if(!gamepad2.b) clawR = false;
                if((gamepad2.y )&& !clawL) {
                    if(ClawServoL.getPosition() == .4) ClawServoL.setPosition(-1);
                    else ClawServoL.setPosition(.4);
                    clawL = true;
                } else if(!gamepad2.y) clawL = false;
                if (gamepad2.dpad_up){
                    checkOne=true;
                }
                if (gamepad2.left_trigger>0){
                    checkTwo=true;
                }



                if(checkOne&&checkTwo){
                    LaunchServo.setPosition(0);
                }


                // Send calculated power to wheels
                FrontLeft.setPower(speedLimit*leftFrontPower);
                FrontRight.setPower(speedLimit*rightFrontPower);
                BackLeft.setPower(speedLimit*leftBackPower);
                BackRight.setPower(speedLimit*rightBackPower);
                Worm.setPower(wormPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());

            }
        }}



