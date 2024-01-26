import tkinter as tk

import operatorFunctionCode as functionCode


class KeybindMaker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("800x800")
        self.root.resizable(False, False)
        self.root.title("Operator Keybind Maker")
        self.list = {}
        self.functions = list(functionCode.functionCode.keys())

        self.keyOptions = ["dpad_down", "dpad_up", "dpad_left", "dpad_right", "left_bumper", "right_bumper", "touchpad",
                           "left_stick_button",
                           "right_stick_button", "circle", "triangle", "square", "cross", "left_trigger",
                           " right_trigger",
                           "left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y"
                           # shouldn't need direct drive
                           ]
        self.boolKeyOptions = ["dpad_down", "dpad_up", "dpad_left", "dpad_right", "left_bumper", "right_bumper",
                               "touchpad",
                               "left_stick_button",
                               "right_stick_button", "circle", "triangle", "square", "cross"]
        self.nonBoolKeyOptions = ["left_trigger", " right_trigger"]
        self.directDriveKeyOptions = ["left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y"]

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
    app = KeybindMaker()
    app.run()
    for key in app.list.keys():
        current_function = key
        cFunc = current_function
        addon = ""
        if app.list.get(cFunc) in app.boolKeyOptions:
            addon = ""
        if app.list.get(cFunc) in app.nonBoolKeyOptions:
            addon = " > 0"
        current_function_code = functionCode.functionCode[current_function]
        current_function_code = current_function_code.replace(" > 0", "")
        current_function_code = current_function_code.replace("placeHolder",
                                                              app.list.get(current_function) + addon)
        print(current_function_code)
    print(
        "if (PastPotent.pastPotentVal != Sensors.getPotentVal(potentiometer)) {\n    calculateFlipPose(lastSetVal, flipServo);\n}\n")
