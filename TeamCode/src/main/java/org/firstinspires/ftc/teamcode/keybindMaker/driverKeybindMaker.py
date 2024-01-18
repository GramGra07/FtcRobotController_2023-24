import tkinter as tk

import driverFunctionCode as functionCode


class fieldCentric:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("300x200")
        self.root.title("Field Centric")

        self.label = tk.Label(self.root, text="Do you want field centric drive?")
        self.label.pack()

        self.yes_button = tk.Button(self.root, text="Yes", command=self.yes)
        self.yes_button.pack()

        self.no_button = tk.Button(self.root, text="No", command=self.no)
        self.no_button.pack()

    def yes(self):
        current_function_code = functionCode.functionCode["fieldCentric"]
        current_function_code = current_function_code.replace("fCenter", "true")
        print(current_function_code)
        self.root.destroy()

    def no(self):
        current_function_code = functionCode.functionCode["fieldCentric"]
        current_function_code = current_function_code.replace("fCenter", "false")
        print(current_function_code)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


class KeybindMaker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("800x800")
        self.root.resizable(False, False)
        self.root.title("Driver Keybind Maker")
        self.list = {}
        self.functions = list(functionCode.functionCode.keys())
        self.functions.remove("fieldCentric")
        if ("lift" in self.functions):
            self.functions.remove("lift")
            self.functions.append("liftUp")
            self.functions.append("liftDown")
        if ("driverAid" in self.functions):
            self.functions.remove("driverAid")
            self.functions.append("Go To Drone Launch")
            self.functions.append("Turn Straight")
            self.functions.append("Turn To Wing")
            self.functions.append("Stop Driver Aid")

        self.keyOptions = ["dpad_down", "dpad_up", "dpad_left", "dpad_right", "left_bumper", "right_bumper", "touchpad",
                           "left_stick_button",
                           "right_stick_button", "circle", "triangle", "square", "cross", "left_trigger",
                           " right_trigger",
                           # "left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y"
                           # shouldn't need direct drive
                           ]
        self.boolKeyOptions = ["dpad_down", "dpad_up", "dpad_left", "dpad_right", "left_bumper", "right_bumper",
                               "touchpad",
                               "left_stick_button",
                               "right_stick_button", "circle", "triangle", "square", "cross"]
        self.nonBoolKeyOptions = ["left_trigger", " right_trigger"]
        # self.directDriveKeyOptions = ["left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y"]

        self.current_function_index = 0
        self.current_function_label = tk.Label(self.root, text=f"Assigning keybind for {self.functions[0]}")
        self.current_function_label.pack()

        for key_option in self.keyOptions:
            button = tk.Button(self.root, text=f"Assign {key_option}")
            button.pack()
            button.config(command=lambda ko=key_option, btn=button: self.assign_keybind(btn))

    def assign_keybind(self, button):
        # get current function
        self.list[self.functions[self.current_function_index]] = button.cget('text').replace("Assign ", "")
        if self.current_function_index < len(self.functions):
            if (self.current_function_index < len(self.functions) - 1):
                self.current_function_label.config(
                    text=f"Assigning keybind for {self.functions[self.current_function_index + 1]}")
            self.current_function_index += 1
            button.config(state='disabled')
        if self.current_function_index == len(self.functions):
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    start_page = fieldCentric()
    start_page.run()
    app = KeybindMaker()
    app.run()
    for key in app.list.keys():
        current_function = key
        cFunc = current_function
        if current_function == "liftUp" or current_function == "liftDown":
            current_function = "lift"
        if current_function == "Go To Drone Launch" or current_function == "Turn Straight" or current_function == "Turn To Wing" or current_function == "Stop Driver Aid":
            current_function = "driverAid"
        addon = ""
        if app.list.get(cFunc) in app.boolKeyOptions:
            addon = ""
        if app.list.get(cFunc) in app.nonBoolKeyOptions:
            addon = " > 0"
        current_function_code = functionCode.functionCode[current_function]
        current_function_code = current_function_code.replace(" > 0", "")
        if (cFunc == "slowMode"):
            current_function_code = current_function_code.replace("placeHolder",
                                                                  app.list.get("slowMode") + addon)
        if cFunc == "plane":
            current_function_code = current_function_code.replace("placeHolder",
                                                                  app.list.get("plane") + addon)
        if cFunc == "liftUp":
            current_function_code = current_function_code.replace("liftUp",
                                                                  app.list.get("liftUp") + addon)
            current_function_code = current_function_code.replace("liftDown",
                                                                  app.list.get("liftDown") + addon)
        if cFunc == "liftDown":
            continue
        if cFunc == "Go To Drone Launch":
            current_function_code = current_function_code.replace("Go To Drone Launch",
                                                                  app.list.get("Go To Drone Launch") + addon)
            current_function_code = current_function_code.replace("Turn Straight",
                                                                  app.list.get("Turn Straight") + addon)
            current_function_code = current_function_code.replace("Turn To Wing",
                                                                  app.list.get("Turn To Wing") + addon)
            current_function_code = current_function_code.replace("Stop Driver Aid",
                                                                  app.list.get("Stop Driver Aid") + addon)
        if cFunc == "Turn Straight" or cFunc == "Turn To Wing" or cFunc == "Stop Driver Aid":
            continue
        print(current_function_code)
