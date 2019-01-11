package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Actual First", group = "Linear")
public class ActualFirst extends LinearOpMode {

    //vars
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private DcMotor motorHR;
    private DcMotor motorHL;

    private DcMotor motorArm;

    //private Servo servo;

    //claw servos
    private Servo clawLeft;
    private Servo clawRight;

    @Override
    public void runOpMode() throws InterruptedException {

        //get config
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorHL = hardwareMap.dcMotor.get("motorHL");
        motorHR = hardwareMap.dcMotor.get("motorHR");

        motorArm = hardwareMap.dcMotor.get("motorArm");

        //servo = hardwareMap.servo.get("servo");

        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        //set modes
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

        //speed var stuff
        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;

        //servo var stuff
        double servoDelta = .02;
        double leftPositionCount = 1;
        double rightPositionCount = 0;

        //lift var stuff
        int liftDelta = 8;
        int liftPositionCountL = 0;
        int liftPositionCountR = 0;

        //arm var stuff
        int armDelta = 12;
        int armPos = 0;

        //reverse movement
        boolean reverse = false;

        //arm power
        double theArmPower = .25; //constant just to hold the power value
        double armPower = .25;
        boolean armPowerOn = true;

        waitForStart();

        while (opModeIsActive()) {
            //drive speed stuff
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

            //reverse stuff
            if (gamepad1.a) {
                reverse = false;
            } else if (gamepad1.b) {
                reverse = true;
            }


            //driving
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



            //move claw
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

            //move linear lift
            if (gamepad2.dpad_up) {
                liftPositionCountL += liftDelta;
                liftPositionCountR += liftDelta;
            } else if (gamepad2.dpad_down) {
                liftPositionCountL -= liftDelta;
                liftPositionCountR -= liftDelta;
            }

            moveLift(liftPositionCountL, liftPositionCountR, 1);

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



            //telemetry stuff
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