package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Servo servo;

    private Servo servoClaw;

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

        servo = hardwareMap.servo.get("servo");

        servoClaw = hardwareMap.servo.get("servoClaw");

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

        motorArm.setPower(.25);

        //speed var stuff
        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;

        //servo var stuff
        double servoDelta = .02;
        double servoPositionCount = 1;

        //lift var stuff
        int liftDelta = 5;
        int liftPositionCountL = 0;
        int liftPositionCountR = 0;

        //arm var stuff
        int armDelta = 5;
        int armPos = 0;

        //reverse movement: if 1 then forward, if -1 then backward
        int reverse = 1;

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
            if (gamepad1.a && reverse == -1) {
                reverse = 1;
            }

            if (gamepad1.b && reverse == 1) {
                reverse = -1;
            }

            //driving
            if(speedConstant == 0)
            {
                motorBL.setPower(-gamepad1.left_stick_y/2 * reverse);
                motorFL.setPower(-gamepad1.left_stick_y/2 * reverse);
                motorBR.setPower(-gamepad1.right_stick_y/2 * reverse);
                motorFR.setPower(-gamepad1.right_stick_y/2 * reverse);
            }

            if(speedConstant == 1)
            {
                motorBL.setPower(-gamepad1.left_stick_y/4 * reverse);
                motorFL.setPower(-gamepad1.left_stick_y/4 * reverse);
                motorBR.setPower(-gamepad1.right_stick_y/4 * reverse);
                motorFR.setPower(-gamepad1.right_stick_y/4 * reverse);
            }

            if(speedConstant == 2)
            {
                motorBL.setPower(-gamepad1.left_stick_y/8 * reverse);
                motorFL.setPower(-gamepad1.left_stick_y/8 * reverse);
                motorBR.setPower(-gamepad1.right_stick_y/8 * reverse);
                motorFR.setPower(-gamepad1.right_stick_y/8 * reverse);
            }


            /*
            //move servo
            if (gamepad2.left_bumper) {
                servoPositionCount -= servoDelta;
            } else if (gamepad2.right_bumper) {
                servoPositionCount += servoDelta;
            }

            /*servoDelta = Range.clip(servoDelta, .2, 1);*/
            //servoClaw.setPosition(servoPositionCount);


            //move linear lift
            if (gamepad2.dpad_up) {
                liftPositionCountL += liftDelta;
                liftPositionCountR += liftDelta;
            } else if (gamepad2.dpad_down) {
                liftPositionCountL -= liftDelta;
                liftPositionCountR -= liftDelta;
            }

            moveLift(liftPositionCountL, liftPositionCountR, 1);

            if (gamepad2.left_bumper) {
                armPos -= armDelta;
            } else if (gamepad2.right_bumper) {
                armPos += armDelta;
            }

            motorArm.setTargetPosition(armPos);

            //telemetry stuff
            int motorBRPosition = motorBR.getCurrentPosition();
            int motorBLPosition = motorBL.getCurrentPosition();

            double motorHLPosition = motorHL.getCurrentPosition();
            double motorHRPosition = motorHR.getCurrentPosition();

            telemetry.addData("motorBR Position: ", motorBRPosition);
            telemetry.addData("motorBL Position: ", motorBLPosition);
            telemetry.addData("The Speed: ", speedConstant);

            telemetry.addData("motorHL Position: ", motorHLPosition);
            telemetry.addData("motorHR Position: ", motorHRPosition);

            telemetry.addData("motorArm position", armPos);

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