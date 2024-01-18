functionCode = {
    "fieldCentric": "fieldCentric = fCenter;",
    "slowMode":
        "if (myOpMode.gamepad1.placeHolder && !slowModeButtonDown && !slowModeIsOn) {\n    slowModeIsOn = true;\n} "
        "else if (myOpMode.gamepad1.placeHolder && !slowModeButtonDown && slowModeIsOn) {\n    slowModeIsOn = "
        "false;\n}\nslowModeButtonDown = myOpMode.gamepad1.placeHolder;\n",
    "plane":
        "if (myOpMode.gamepad1.placeHolder && !planeButtonDown && !planeReleased) {\n    ServoUtil.releaseAirplane("
        "airplaneServo);\n    planeReleased = true;\n} else if (myOpMode.gamepad1.placeHolder && !planeButtonDown && "
        "planeReleased) {\n    resetAirplane(airplaneServo);\n    planeReleased = false;\n}\nplaneButtonDown = "
        "myOpMode.gamepad1.placeHolder;\n",
    "lift":
        "if (myOpMode.gamepad1.liftUp) {\n    liftPower = liftMax;\n} else if (myOpMode.gamepad1.liftDown) {"
        "\n    liftPower = -liftMax;\n} else {\n    liftPower = 0;\n}\n",
    "driverAid":
        "doDriverAid(drive, myOpMode.gamepad1.Go To Drone Launch, myOpMode.gamepad1.Turn Straight, "
        "myOpMode.gamepad1.Turn To Wing, myOpMode.gamepad1.Stop Driver Aid);",
}
