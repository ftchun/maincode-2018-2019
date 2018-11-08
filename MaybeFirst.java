package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Maybe First", group = "Linear")
public class MaybeFirst extends LinearOpMode {

    //vars
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private DcMotor motorHR;
    private DcMotor motorHL;

    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        //get config
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorHL = hardwareMap.dcMotor.get("motorHL");
        motorHR = hardwareMap.dcMotor.get("motorHR");

        servo = hardwareMap.servo.get("servo");

        //set modes
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorHR.setDirection(DcMotor.Direction.REVERSE);
        motorHL.setDirection(DcMotor.Direction.REVERSE);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //servo var stuff
        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;

        double servoDelta = .02;
        double servoPositionCount = 1;

        int liftDelta = 10;
        int liftPositionCountL = 0;
        int liftPositionCountR = 0;

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

            //driving
            if(speedConstant == 0)
            {
                motorBL.setPower(-gamepad1.left_stick_y/2);
                motorFL.setPower(-gamepad1.left_stick_y/2);
                motorBR.setPower(-gamepad1.right_stick_y/2);
                motorFR.setPower(-gamepad1.right_stick_y/2);
            }

            if(speedConstant == 1)
            {
                motorBL.setPower(-gamepad1.left_stick_y/4);
                motorFL.setPower(-gamepad1.left_stick_y/4);
                motorBR.setPower(-gamepad1.right_stick_y/4);
                motorFR.setPower(-gamepad1.right_stick_y/4);
            }

            if(speedConstant == 2)
            {
                motorBL.setPower(-gamepad1.left_stick_y/8);
                motorFL.setPower(-gamepad1.left_stick_y/8);
                motorBR.setPower(-gamepad1.right_stick_y/8);
                motorFR.setPower(-gamepad1.right_stick_y/8);
            }

            //move servo
            if (gamepad2.left_bumper) {
                servoPositionCount -= servoDelta;
            } else if (gamepad2.right_bumper) {
                servoPositionCount += servoDelta;
            }

            servoDelta = Range.clip(servoDelta, .2, 1);
            servo.setPosition(servoPositionCount);

            //move linear lift
            if (gamepad2.dpad_up) {
                liftPositionCountL += liftDelta;
                liftPositionCountR += liftDelta;
            } else if (gamepad2.dpad_down) {
                liftPositionCountL -= liftDelta;
                liftPositionCountR -= liftDelta;
            }

            liftPositionCountL = Range.clip(liftPositionCountL, 0, 1100);
            liftPositionCountR = Range.clip(liftPositionCountR, 0, 1000);

            moveLift(liftPositionCountL, liftPositionCountR, 1);

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

            telemetry.update();
        }

        idle();

    }

    public void moveLift(int positionL, int positionR, double power) {
        motorHL.setTargetPosition(positionL);
        motorHR.setTargetPosition(positionR);
        motorHL.setPower(power);
        motorHR.setPower(power);
    }

}
