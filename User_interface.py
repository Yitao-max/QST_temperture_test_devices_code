import tkinter as tk
from tkinter import messagebox
import re
import serial
import time
import threading
import xlwt

# path
file_path = 'Your arduino ino file path'
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
EXCEL_FILE_PATH = 'Your output excel file path'

# read code
def read_arduino_code(file_path):
    with open(file_path, 'r') as file:
        return file.read()

# change Setpoint
def modify_setpoint_value(code, new_value):
    pattern = r'(Setpoint_first\s*=\s*.*;)'
    new_code = re.sub(pattern, f'Setpoint_first = {new_value};', code ,1)
    return new_code

# save code
def save_arduino_code(file_path, code):
    with open(file_path, 'w') as file:
        file.write(code)

#port comm 
def send_temperature_mode_and_read_serial(mode_value, new_value):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.close()
        ser.open()
        time.sleep(2)  # waitting for port setting up
        ser.write(f"{mode_value}\n".encode('utf-8'))
        ser.write(f"{new_value}\n".encode('utf-8'))
        #response = ser.readline().decode().strip()  # read Arduino response
        #print(f"Arduino response: {response}")  # print Arduino response

        workbook = xlwt.Workbook()
        sheet = workbook.add_sheet('Data')

        # excel title
        sheet.write(0, 0, 'Time')
        sheet.write(0, 1, 'Temp')

        # read data
        row = 1
        start_time = time.time()
        flag = 1
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode().strip()
                if line:
                    elapsed_time = time.time() - start_time
                    print(f"Time: {elapsed_time}, Temp: {line}")
                    if "=" in line:
                        if flag == 0:
                            break
                        if elapsed_time > 15:
                            break
                    else:
                     flag = 0
                     sheet.write(row, 0, elapsed_time)
                     sheet.write(row, 1, line)
                     row += 1
        workbook.save(EXCEL_FILE_PATH)
        ser.close()
        messagebox.showinfo("Success", "temperatureMode and Setpoint value have been transformed to Arduino")
    except Exception as e:
        messagebox.showerror("error", f"can't deliver temperatureMode value：{e}")

# save changes
def save_changes():
    new_value = value_entry.get()
    mode_value = mode_var.get()

    if not new_value:
        messagebox.showerror("error", "The value field is required")
        return

    try:
        new_value = float(new_value)  
    except ValueError:
        messagebox.showerror("error", "The value must be a number")
        return

    try:
        code = read_arduino_code(file_path)
        modified_code = modify_setpoint_value(code, new_value)
        save_arduino_code(file_path, modified_code)
        messagebox.showinfo("Success", "The Setpoint value has been updated and saved.")

        serial_thread = threading.Thread(target=send_temperature_mode_and_read_serial, args=(mode_value, new_value))
        serial_thread.start()        
    except Exception as e:
        messagebox.showerror("error", f"Error occurred：{e}")    
        
# GUI
root = tk.Tk()
root.title("Arduino Temperature-sensor config")

tk.Label(root, text="Setpoint New value:").grid(row=0, column=0)
value_entry = tk.Entry(root, width=50)
value_entry.grid(row=0, column=1)

mode_var = tk.IntVar()
tk.Checkbutton(root, text="temperatureMode = rise", variable=mode_var, onvalue=1, offvalue=0).grid(row=1, column=0)
tk.Checkbutton(root, text="temperatureMode = decline", variable=mode_var, onvalue=2, offvalue=0).grid(row=1, column=1)

save_button = tk.Button(root, text="Save change", command=save_changes)
save_button.grid(row=2, column=0, columnspan=2)

root.mainloop()

