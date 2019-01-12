package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
*  This is our main TeleOp program, aptly titled 'Actual First'. Its main functions are to
*  drive the robot and operate its peripheral extensions. These comments will take you through
*  the functions of each section of code that we wrote.
*/

@TeleOp(name = "Actual First", group = "Linear")
public class ActualFirst extends LinearOpMode {

    /*
    * Here we declare our main motors that are on the robot. The first four are the driving
    * motors: FL corresponds to the front-left motor, BR corresponds to back-right, etc.
    * The next two are the Hex motors, left and right. These are used to control the
    * linear lift. Next is the arm motor, for pivoting the arm. Finally there are the two
    * servos for the claw on the arm, left and right as well.
    */

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private DcMotor motorHR;
    private DcMotor motorHL;

    private DcMotor motorArm;

    private Servo clawLeft;
    private Servo clawRight;

    @Override
    public void runOpMode() throws InterruptedException {


        /*
        * Next is getting the motors from the configuration on the Android phone. We made the
        * configuration names the same as the program names for better convention and
        * easier management.
        */

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorHL = hardwareMap.dcMotor.get("motorHL");
        motorHR = hardwareMap.dcMotor.get("motorHR");

        motorArm = hardwareMap.dcMotor.get("motorArm");

        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");


        /*
        * Now we have to set the modes for the motors. Certain motors require that they be
        * reversed, as to simplify their programming for when they are facing the opposite
        * direction as the others. The motors with encoders are then reset, as to
        * clean their memory and set the value to zero. Then they individually are set
        * to the mode we need them in. The operating motors like the ones for the lift
        * use the encoders, while the driving motors do not.
        */

        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorHR.setDirection(DcMotor.Direction.REVERSE);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /*
        * Next are the bank of variables that we initialize, which will be utilized later
        * in the program. Here is a brief description of each:
        * Speed change: adjusts the driving speed of the robot
        * Servo vars: values for the servos' movement and position
        * Lift vars: values for the lift's movement and position
        * Arm vars: values for the arm's movement and position
        * Reverse movement: boolean for whether or not the bot is in reverse mode
        * Arm power: values for the arm's power
        */

        //speed change
        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;

        //servo vars
        double servoDelta = .02;
        double leftPositionCount = 1;
        double rightPositionCount = 0;

        //lift vars
        int liftDelta = 8;
        int liftPositionCountL = 0;
        int liftPositionCountR = 0;

        //arm vars
        int armDelta = 12;
        int armPos = 0;

        //reverse movement
        boolean reverse = false;

        //arm power
        double theArmPower = .25; //constant just to hold the power value
        double armPower = .25; //actual arm power
        boolean armPowerOn = true;



        waitForStart();



        while (opModeIsActive()) {

            /*
            * First inside the main loop of the program, we have blocks controlling the
            * drive modes of the robot. Speed change checks if gamepad1's 'x' or 'y' buttons are
            * pressed. When 'x' is pressed, it increases the speed constant, and 'y' decreases
            * it. Reverse checks if gamepad1's 'b' button is pressed. If so, it sets the
            * 'reverse' boolean to true, while 'a' sets it to false.
            */

            //speed change
            if(prevY == false && gamepad1.y && speedConstant>0)
            {
                speedConstant -= 1;
            }
            if(prevX == false && gamepad1.x && speedConstant<2)
            {
                speedConstant += 1;
            }

            prevX = gamepad1.x;
            prevY = gamepad1.y;


            //reverse
            if (gamepad1.a) {
                reverse = false;
            } else if (gamepad1.b) {
                reverse = true;
            }



            /*
            * Next is the main driving control block. It is three main if statements to check
            * what the speed constant is currently set to. The lower the speed constant, the
            * faster the robot moves. Inside each main if statement there is another to
            * check whether reverse is active. It then determines how the motors should
            * move, depending on the speed constant and reverse. Driving is controlled using
            * gamepad1's joysticks.
            */

            if(speedConstant == 0)
            {
                if (!reverse) {
                    motorBL.setPower(-gamepad1.left_stick_y/2);
                    motorFL.setPower(-gamepad1.left_stick_y/2);
                    motorBR.setPower(-gamepad1.right_stick_y/2);
                    motorFR.setPower(-gamepad1.right_stick_y/2);
                } else if (reverse) {
                    motorBR.setPower(gamepad1.left_stick_y/2);
                    motorFR.setPower(gamepad1.left_stick_y/2);
                    motorBL.setPower(gamepad1.right_stick_y/2);
                    motorFL.setPower(gamepad1.right_stick_y/2);
                }
            }

            if(speedConstant == 1)
            {
                if (!reverse) {
                    motorBL.setPower(-gamepad1.left_stick_y/4);
                    motorFL.setPower(-gamepad1.left_stick_y/4);
                    motorBR.setPower(-gamepad1.right_stick_y/4);
                    motorFR.setPower(-gamepad1.right_stick_y/4);
                } else if (reverse) {
                    motorBR.setPower(gamepad1.left_stick_y/4);
                    motorFR.setPower(gamepad1.left_stick_y/4);
                    motorBL.setPower(gamepad1.right_stick_y/4);
                    motorFL.setPower(gamepad1.right_stick_y/4);
                }
            }

            if(speedConstant == 2)
            {
                if (!reverse) {
                    motorBL.setPower(-gamepad1.left_stick_y/8);
                    motorFL.setPower(-gamepad1.left_stick_y/8);
                    motorBR.setPower(-gamepad1.right_stick_y/8);
                    motorFR.setPower(-gamepad1.right_stick_y/8);
                } else if (reverse) {
                    motorBR.setPower(gamepad1.left_stick_y/8);
                    motorFR.setPower(gamepad1.left_stick_y/8);
                    motorBL.setPower(gamepad1.right_stick_y/8);
                    motorFL.setPower(gamepad1.right_stick_y/8);
                }
            }


            /*
            * Here we have the control for our claw end effector. gamepad2's left and right
            * bumper buttons control the claw's servos. They increase and decrease the servos'
            * position count variables, at a rate defined by 'servoDelta'. They are clipped
            * between 0 and 1 as to not go out of the servo's range, then the servos
            * are set to those positions.
            */

            //claw control
            if (gamepad2.left_bumper) {
                rightPositionCount -= servoDelta;
                leftPositionCount += servoDelta;
            } else if (gamepad2.right_bumper) {
                rightPositionCount += servoDelta;
                leftPositionCount -= servoDelta;
            }

            leftPositionCount = Range.clip(leftPositionCount, 0, 1);
            rightPositionCount = Range.clip(rightPositionCount, 0, 1);

            clawLeft.setPosition(leftPositionCount);
            clawRight.setPosition(rightPositionCount);


            /*
            * The claw is mounted on the arm, which is coded below. First is a block for an
            * emergency power cut to the arm, controlled with gamepad2's 'b' button' the 'a'
            * button returns power. This uses the 'armPowerOn' boolean variable. The arm is
            * controlled using gamepad2's 'x' and 'y' buttons, 'x' lowering and 'y' raising.
            * Arm power uses 'armPos' for position and 'armDelta' for movement speed.
            */

            //arm emergency stop
            if (gamepad2.b) {
                armPowerOn = false;
            } else if (gamepad2.a) {
                armPowerOn = true;
            }

            //arm control
            if (armPowerOn) {
                armPower = theArmPower;
                if (gamepad2.x) {
                    armPos -= armDelta;
                } else if (gamepad2.y) {
                    armPos += armDelta;
                }
            } else if (!armPowerOn) {
                armPower = 0;
            }

            motorArm.setPower(armPower);
            motorArm.setTargetPosition(armPos);


            /*
            * The next block of code controls the linear lift, which uses the REV hex motors.
            * Just like the arm and claw, the lifts' positions are counted with their own
            * position count variables and changed with 'liftDelta'. Then the method
            * 'moveLift()' is called to set the motors' positions to the position count
            * variable values and setting their powers to 1.
            */

            if (gamepad2.dpad_up) {
                liftPositionCountL += liftDelta;
                liftPositionCountR += liftDelta;
            } else if (gamepad2.dpad_down) {
                liftPositionCountL -= liftDelta;
                liftPositionCountR -= liftDelta;
            }

            moveLift(liftPositionCountL, liftPositionCountR, 1);


            /*
            * Finally we have the telemetry. This is a display just for our coach to see
            * where our values are at. The positions of all the motors with encoders are
            * displayed, along with the speed constant.
            */

            int motorBRPosition = motorBR.getCurrentPosition();
            int motorBLPosition = motorBL.getCurrentPosition();

            double motorHLPosition = motorHL.getCurrentPosition();
            double motorHRPosition = motorHR.getCurrentPosition();

            double clawLeftPos = clawLeft.getPosition();
            double clawRightPos = clawRight.getPosition();

            telemetry.addData("motorBR Position: ", motorBRPosition);
            telemetry.addData("motorBL Position: ", motorBLPosition);
            telemetry.addData("The Speed: ", speedConstant);

            telemetry.addData("motorHL Position: ", motorHLPosition);
            telemetry.addData("motorHR Position: ", motorHRPosition);

            telemetry.addData("motorArm position", armPos);

            telemetry.addData("clawLeft pos", clawLeftPos);
            telemetry.addData("clawRight pos", clawRightPos);

            telemetry.update();
        }

        idle();

    }

    //method for moving linear lift
    public void moveLift(int positionL, int positionR, double power) {
        motorHL.setTargetPosition(positionL);
        motorHR.setTargetPosition(positionR);
        motorHL.setPower(power);
        motorHR.setPower(power);
    }

}