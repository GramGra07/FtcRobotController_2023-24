people = ["chase", "camden", "kian", "grady", "michael", "test"]
keyOptions = ["dpad_down", "dpad_up", "dpad_left", "dpad_right", "left_bumper", "right_bumper", "left_stick_button",
              "right_stick_button", "circle", "triangle", "square", "cross", "left_trigger", " right_trigger",
              "left_stick_x",
              "left_stick_y", "right_stick_x", "right_stick_y"]
driveOptions = ["fieldCentric", "robotCentric", "slowmode"]
operatorOptions = ["openClaw", "closeClaw", "airplaneLauncher"]
highButtons = keyOptions[0:11]
boundKeys = []
person = input("Who is setting up their keybinds? ").lower()
while not person in people:
    print("That person does not exist")
    person = input("Who is setting up their keybinds? ").lower()

drive = input("Are you setting up driver (d) or operator (o) keybinds? ").lower()
while not drive == "d" and not drive == "o":
    print("That is not a valid option")
    drive = input("Are you setting up driver (d) or operator (o) keybinds? ").lower()

if drive == "d":
    print(" ")
    for line in driveOptions[0:2]:
        print(line)
    print(" ")
    print("For the future type the first letter of the thing you want to bind except if it is a button")
    driveMode = input("Which drive mode to you want? ").lower()
    while not driveMode == "f" and not driveMode == "r":
        print("That is not a valid option")
        driveMode = input("Which drive mode to you want? ").lower()
    if driveMode == "f":
        driveMode = "true"
    elif driveMode == "r":
        driveMode = "false"

    print("Which button do you want for slowmode on/off? Please make it exact ")
    print(keyOptions[0:12])

    slowMode = input(" ").lower()
    while not slowMode in keyOptions[0:12]:
        print("That is not a valid option")
        slowMode = input(" ").lower()

    print(" ")
    print("Enter the following into the keybindings in Drivers.java for "+person[0].upper()+person[1:]+":")
    print(" ")
    print("fieldCentric = "+ driveMode)
    if slowMode in highButtons:
        if "dpad" in slowMode:
            var = slowMode[0] + slowMode[5:] +"High"
        if "stick" in slowMode:
            var = slowMode[0]+"Stick"+slowMode[-6:] +"High"
        if "bumper" in slowMode:
            var = slowMode[0]+"Bumper High"
        else:
            var = slowMode +"High"
        print("public static boolean "+var+" = false;")
        print("// This goes above init")
        print("if (myOpMode.gamepad1."+slowMode+" && !"+var+" && !slowModeIsOn) {")
        print("     slowModeIsOn = true;")
        print("else if (myOpMode.gamepad1."+slowMode+" && !"+var+" && slowModeIsOn) {")
        print("     slowModeIsOn = false;")
        print("}")
        print(var+" = myOpMode.gamepad1."+slowMode+";")
if drive == "o":
    print(' ')
    for line in operatorOptions:
        print(line)
    print(' ')
    print("For the future type the first letter of the thing you want to bind except if it is a button")
    option = []
    button = ["", "", "", "", ""]
    for i in range(5):
        if i == 0:
            print("Bind the open claw 1 button")
        elif i == 1:
            print("Bind the close claw 1 button")
        elif i == 3:
            print("Bind the open claw 2 button")
        elif i == 4:
            print("Bind the close claw 2 button")
        elif i == 2:
            print("Bind the airplane launcher button")
        print("Which button do you want to bind it to? Please make it exact ")
        print(keyOptions[0:12])
        button[i] = input(" ").lower()
        while not button[i] in keyOptions[0:12]:
            print("That is not a valid option")
            button[i] = input(" ").lower()

        keyOptions.remove(button[i])
        print(" ")

    print(" ")
    print("Enter the following into the keybindings in Operators.java for "+person[0].upper()+person[1:]+":")
    print(" ")
    print("if (myOpMode.gamepad2."+button[2]+" && airplaneArmed){")
    print("     airplanePower = airplaneMax;")
    print("}else{")
    print("     airplanePower = 0;")
    print("}")
    print("if (myOpMode.gamepad2."+button[0]+"){")
    print("openClaw(claw1);")
    print("}")
    print("if (myOpMode.gamepad2."+button[1]+"){")
    print("     closeClaw(claw1);")
    print("}")
    print("if (myOpMode.gamepad2."+button[3]+"){")
    print("     openClaw(claw2);")
    print("}")
    print("if (myOpMode.gamepad2."+button[4]+"){")
    print("     closeClaw(claw2);")
    print("}")