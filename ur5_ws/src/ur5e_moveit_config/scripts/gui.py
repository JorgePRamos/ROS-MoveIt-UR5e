#!/usr/bin/env python

import time
import rospy
import threading
import tkinter as tk

from tkinter import *
from PIL import Image, ImageTk
from std_msgs.msg import String
from tkinter.messagebox import *
from time import strftime, localtime



def value_check(v):

    try:
        # If the value is convertable to integer, convert it
        return int(v)
    except: 
        # If not, ignore and go ahead
        pass
    try:
        # If the value is convertable to float, convert it
        return float(v)
    except:
        if v == 'false': 
            return False
        elif v == 'true': 
            return True
        else:
            return str(v[1:-1])


def str_to_dict(s): 
    d = {}
    s = s[1:-1].split(", ")

    for item in s: 
        b = item.split(": ")
        key = b[0][1:-1]
        value = value_check(b[1])
        
        d[key] = value

    return d


class PostaClientStatus: 
    def __init__(self, frame, row=0, col=0):

        self.label = tk.Label(frame, text = "Posta Client: ", bg = bg_color, fg='white', pady=y_sm_pad, font=btn_font)
        self.label.grid(row=row, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.status = tk.Label(frame, text = "Connecting", bg = bg_color, fg='yellow', pady=y_sm_pad, font=btn_font)
        self.status.grid(row=row, column=col+1, padx=x_lg_pad, sticky= tk.W + tk.E)

    def change_status(self, status):
        if status: 
            self.status.configure(text='Online', fg='green')
        else: 
            self.status.configure(text='Offline', fg='red')


class DrawerButton: 

    def __init__(self, buttonframe, pub, num, row=2, state='CLOSED'):

        if state != 'CLOSED' and state != 'OPEN':
            raise KeyError('Button state can be only OPEN or CLOSED')

        text = 'DRAWER {0}\n{1}'.format(num, state)

        self.num = num
        self.state = state

        if self.state == 'OPEN': 
            self.color = 'GREEN'
        else: 
            self.color = 'RED'

        self.button = tk.Button(buttonframe, text=text, font=btn_font, bg=btn_bg_color, fg=self.color, command=self.button_action)
        self.button.grid(row=row, column=num-1, padx=x_sm_pad, sticky= tk.W + tk.E)

        self.pub = pub

    
    def button_action(self):
        # If button is pressed, send the action to sud_posta_command
        if self.state == 'OPEN':
            # Publisher Command 'Action-p1-p2'
            cmd_str = 'CloseDrawer|{}'.format(self.num)
            self.pub.publish(cmd_str)

        else:
            cmd_str = 'OpenDrawer|{}'.format(self.num)
            self.pub.publish(cmd_str)

    def animation(self):
        c = 0
        while not new_state['robot'] == 'moving':
            # Waiting the robot to start moving
            pass

        while new_state['robot'] == 'moving':
            if c == 0:
                self.button.configure(fg=btn_bg_color, bg=btn_fg_color)
                c = 1
            else: 
                self.button.configure(bg=btn_bg_color, fg=btn_fg_color)
                c = 0
            time.sleep(0.5)
        
    def button_callback(self, new_state):
        # Changes the state of the button when invoced
        if new_state == 'open':
            new_text = 'DRAWER {0}\nOPEN'.format(self.num)
            self.button.configure(text=new_text, fg='GREEN')
            self.state = 'OPEN'
        else:
            new_text = 'DRAWER {0}\nCLOSED'.format(self.num)
            self.button.configure(text=new_text, fg='RED')
            self.state = 'CLOSED'

    def animation(self):
        c = 0
        while not new_state['robot'] == 'moving':
            # Waiting the robot to start moving
            pass

        while new_state['robot'] == 'moving':
            if c == 0:
                self.button.configure(fg=btn_bg_color, bg=btn_fg_color)
                c = 1
            else: 
                self.button.configure(bg=btn_bg_color, fg=btn_fg_color)
                c = 0
            time.sleep(0.5)
        
        # Back to original color
        self.button.configure(bg=btn_bg_color, fg=btn_fg_color)

class DrawersSection:

    def __init__(self, frame, pub, row=1, col=0):

        self.label = tk.Label(frame, text = "Drawers Status", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font)
        self.label.grid(row=row, column=col, columnspan=6, padx=x_sm_pad, sticky= tk.W + tk.E)
        
        self.btn1 = DrawerButton(frame, pub, num=1, row=row+1)
        self.btn2 = DrawerButton(frame, pub, num=2, row=row+1)
        self.btn3 = DrawerButton(frame, pub, num=3, row=row+1)
        self.btn4 = DrawerButton(frame, pub, num=4, row=row+1)
        self.btn5 = DrawerButton(frame, pub, num=5, row=row+1)
        self.btn6 = DrawerButton(frame, pub, num=6, row=row+1)

        self.drawers_button_list = [self.btn1, self.btn2, self.btn3, self.btn4, self.btn5, self.btn6]
    
    def open_close_drawer_num(self, num, new_state):
        self.drawers_button_list[num-1].button_callback(new_state)

class TimeDisplay:
    def __init__(self, frame, row=1, col=11):

        self.label = tk.Label(frame, text = "Date & Time", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font)
        self.label.config(anchor=CENTER)
        self.label.grid(row=row, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.global_time = tk.Label(frame, text = "", bg = bg_color, fg=title_color, font=info_font)
        self.global_time.config(anchor=CENTER)
        self.global_time.grid(row=row+1, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

    def change_time(self, time):
        self.global_time.configure(text=time)

class PouchesDisplay:

    def __init__(self, frame, row=1, col=7):
        # Pouches Count
        self.label = tk.Label(frame, text = "Pouches Detected", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font, anchor=CENTER)
        self.label.grid(row=row, column=col, columnspan=3, padx=x_lg_pad, pady=y_sm_pad, sticky= tk.W + tk.E)
        
        self.pouches_count = tk.Label(frame, text = " ", bg = bg_color, fg='green', font=num_pouch_font, anchor=CENTER)
        self.pouches_count.grid(row=row+1, column=col, columnspan=3, padx=x_lg_pad, sticky= tk.W + tk.E)

    def change_num_pouches(self, num):
        if num > 10: 
            color = 'green'
        elif num > 3: 
            color = 'yellow'
        else: 
            color = 'red'
        self.pouches_count.configure(text=str(num), fg=color)



class DrawerImagingButton: 
    def __init__(self, frame, pub, num, row, col):

        self.button = tk.Button(frame, text=str(num), font=btn_font, bg=btn_bg_color, fg=btn_fg_color, command=self.button_action)
        self.button.grid(row=row, column=col+num-1, rowspan=2, padx=x_sm_pad, sticky= tk.W + tk.E)

        self.pub = pub
        self.num = num
    
    def button_action(self):
        self.pub.publish('GoToDrawerImaging|{0}'.format(self.num))

class GoToDrawerImaging: 
    def __init__(self, frame, pub, row, col):
        self.label = tk.Label(frame, text = "Go to Drawer Imaging", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font)
        self.label.grid(row=row, column=col, columnspan=6, padx=x_sm_pad, sticky= tk.W + tk.E)
        
        # di stands for drawer imaging
        self.di_1 = DrawerImagingButton(frame, pub, num=1, row=row+1, col=col)
        self.di_2 = DrawerImagingButton(frame, pub, num=2, row=row+1, col=col)
        self.di_3 = DrawerImagingButton(frame, pub, num=3, row=row+1, col=col)
        self.di_4 = DrawerImagingButton(frame, pub, num=4, row=row+1, col=col)
        self.di_5 = DrawerImagingButton(frame, pub, num=5, row=row+1, col=col)
        self.di_6 = DrawerImagingButton(frame, pub, num=6, row=row+1, col=col)

class ScaleDisplay:

    def __init__(self, frame, pub, row=3, col=7):

        self.pub = pub

        # Scale Label
        self.label = tk.Label(frame, text = "Scale", bg = bg_color, fg='white', pady=title_y_pad, font=title_font)
        self.label.config(anchor=CENTER)
        self.label.grid(row=row, column=col, columnspan=5, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Weight
        self.weight_lab = tk.Label(frame, text = "Weight", bg = bg_color, fg='white', pady=y_sm_pad, font=btn_font)
        self.weight_lab.config(anchor=CENTER)
        self.weight_lab.grid(row=row+1, column=col, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.scale_weight = tk.Label(frame, text = "--- g", bg = bg_color, fg='white', pady=y_sm_pad, font=title_font)
        self.scale_weight.config(anchor=CENTER)
        self.scale_weight.grid(row=row+2, column=col, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Weight Time Stamp
        self.weight_time_lab = tk.Label(frame, text = "Time Stamp", bg = bg_color, pady=y_sm_pad, fg='white', font=btn_font)
        self.weight_time_lab.config(anchor=CENTER)
        self.weight_time_lab.grid(row=row+1, column=col+2, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.weight_time = tk.Label(frame, text = "---", bg = bg_color, fg='white', pady=y_sm_pad, font=btn_font)
        self.weight_time.config(anchor=CENTER)
        self.weight_time.grid(row=row+2, column=col+2, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Reset Button
        self.reset_scale = tk.Button(frame, text="Reset", bg=btn_bg_color, fg=btn_fg_color, pady=y_sm_pad, font=btn_font, command=self.reset)
        self.reset_scale.grid(row=row+1, column=col+4, rowspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

    def reset(self):
        cmd_str = 'SetScaleToZero'
        self.pub.publish(cmd_str)
        self.weight_time.configure(text='-----')

    def change_weight(self, value):
        self.scale_weight.configure(text=str(value)+' g')
    
    def change_time(self, time):
        self.weight_time.configure(text=str(time))



class GripperButton: 

    def __init__(self, frame, pub, row=4, col=1, state='OPEN'):

        if state != 'CLOSED' and state != 'OPEN':
            raise KeyError('Button state can be only OPEN or CLOSED')

        text = 'GRIPPER\n{0}'.format(state)
        self.state = state

        if self.state == 'OPEN': 
            self.color = 'GREEN'
        else: 
            self.color = 'RED'

        self.button = tk.Button(frame, text=text, font=btn_font, bg=btn_bg_color, fg=self.color, command=self.button_action)
        self.button.grid(row=row, column=col, padx=x_sm_pad, rowspan=2, sticky= tk.W + tk.E)
    
        self.pub = pub
    
    def button_action(self):
        # If button is pressed, send the action to sud_posta_command
        if self.state == 'OPEN':
            # Publisher Command 'Action-p1-p2'
            cmd_str = 'CloseGripper'
            self.pub.publish(cmd_str)
        else:
            cmd_str = 'OpenGripper'
            self.pub.publish(cmd_str)

    def button_callback(self, new_gripper_aperture):
        # Changes the state of the button when invoced
        # When Gripper is open
        if new_gripper_aperture > 50:
            new_text = 'GRIPPER\nOPEN'
            self.button.configure(text=new_text, fg='GREEN')
            self.state = 'OPEN'
        else:
            new_text = 'GRIPPER\nCLOSED'
            self.button.configure(text=new_text, fg='RED')
            self.state = 'CLOSED'

class GripperApertureSetter:
    def __init__(self, frame, pub, col=3, row=4):
        
        self.pub = pub

        self.title_label = tk.Label(frame, text = "Gripper Aperture", bg = bg_color, fg='white', font=14)
        self.title_label.grid(row=row, column=col, columnspan=2, padx=x_sm_pad, pady = y_sm_pad, sticky= tk.W + tk.E)

        self.text_box = tk.Text(frame, bg = 'white', fg='black', font=btn_font, height=1, width=6)
        self.text_box.grid(row=row+1, column=col, padx=x_sm_pad)

        self.set_button = tk.Button(frame, text='Set', bg=btn_bg_color, fg=btn_fg_color, font=btn_font, command=self.button_action)
        self.set_button.grid(row=row+1, column=col+1, padx=x_sm_pad, sticky= tk.W + tk.E)
    
    def change_value(self, value):
        self.text_box.delete("1.0","end")
        self.text_box.insert(tk.END, str(value))
    
    def button_action(self):
        # If pressed, has to get the value from text box and SetGripperAperture
        new_value = self.text_box.get("1.0","end-1c")
        try:
            new_value = int(new_value)

            if new_value <= 86 and new_value >= 0:

                cmd_str = 'SetGripperApertureMM|{0}'.format(new_value)
                self.pub.publish(cmd_str)
            else:
                showerror('ERROR', 'Gripper Aperture Range\nnot admissible\n(range: 0-86)')
        except:
            showerror('ERROR', 'Gripper Aperture Value\nnot admissible\n(need int number)')

class PouchInGripper:
    def __init__(self, frame, col=2, row=4):
        self.label = tk.Label(frame, text = "NO POUCH\nIN GRIPPER", bg = bg_color, fg='red', font=btn_font)
        self.label.grid(row=row, column=col, rowspan=2, padx=x_sm_pad, sticky= tk.W + tk.E)

    def change_value(self, value): 
        if value: 
            self.label.config(text='POUCH\nIN GRIPPER', fg='green')
        else:
            self.label.config(text='NO POUCH\nIN GRIPPER', fg='red')

class Mark10Section: 
    def __init__(self, frame, pub, row, col):
        self.pub = pub

        # Scale Label
        self.label = tk.Label(frame, text = "Mark 10", bg = bg_color, fg='white', pady=title_y_pad, font=title_font, anchor=CENTER)
        self.label.grid(row=row, column=col, columnspan=5, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Instrument Ready
        self.ready_lab = tk.Label(frame, text = "Instrument Ready", bg = bg_color, fg='white', pady=y_sm_pad, font=subtitle_font, anchor=CENTER)
        self.ready_lab.grid(row=row+1, column=col, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.ready = tk.Label(frame, text = "", bg = bg_color, fg='white', pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.ready.grid(row=row+2, column=col, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Plate Distance
        self.plate_dist_lab = tk.Label(frame, text = "Plate Distance", bg = bg_color, fg='white', pady=y_sm_pad, font=subtitle_font)
        self.plate_dist_lab.config(anchor=CENTER)
        self.plate_dist_lab.grid(row=row+1, column=col+2, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.plate_dist = tk.Label(frame, text = "-1", bg = bg_color, fg='white', pady=y_sm_pad, font=btn_font)
        self.plate_dist.config(anchor=CENTER)
        self.plate_dist.grid(row=row+2, column=col+2, columnspan=2, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Test Completed
        self.test_status_lab = tk.Label(frame, text = "Test Status", bg = bg_color, fg=subtitle_fg_color, pady=y_sm_pad, font=subtitle_font)
        self.test_status_lab.config(anchor=CENTER)
        self.test_status_lab.grid(row=row+1, column=col+4, columnspan=1, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.test_status = tk.Label(frame, text = "   COMPLETED   ", bg = bg_color, fg='green', pady=y_sm_pad, font=btn_font)
        self.test_status.config(anchor=CENTER)
        self.test_status.grid(row=row+2, column=col+4, columnspan=1, padx=x_sm_pad, sticky= tk.W + tk.E)

    
    def change_test_status(self, state):
        if state:
            self.test_status.configure(text='   COMPLETED   ', fg='green')
        else: 
            self.test_status.configure(text='NOT COMPLETED', fg='red')

    def change_plate_distance(self, dist):
        self.plate_dist.configure(text=str(dist))

    def change_mark10_ready(self, state):

        if state: 
            self.ready.configure(text='READY', fg='green')
        else: 
            self.ready.configure(text='NOT READY', fg='red')



class ActionButton:
    def __init__(self, frame, pub, text, cmd_str, col, row):
        self.button = tk.Button(frame, text=text, font=btn_font, bg=btn_bg_color, fg=btn_fg_color, command=self.button_action)
        self.button.grid(row=row, column=col, rowspan=2, padx=x_sm_pad, pady= y_sm_pad, sticky= tk.W + tk.E)

        self.cmd_str = cmd_str
        self.pub = pub
        
    def button_action(self):
        self.pub.publish(self.cmd_str)
        
    def animation(self):
        c = 0
        while not new_state['robot'] == 'moving':
            # Waiting the robot to start moving
            pass

        while new_state['robot'] == 'moving':
            if c == 0:
                self.button.configure(fg=btn_bg_color, bg=btn_fg_color)
                c = 1
            else: 
                self.button.configure(bg=btn_bg_color, fg=btn_fg_color)
                c = 0
            time.sleep(0.5)
        
        # Back to original color
        self.button.configure(bg=btn_bg_color, fg=btn_fg_color)

class ActionSection: 
    def __init__(self, frame, pub, row=10, col=1):
        self.label = tk.Label(frame, text = "Actions", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font)
        self.label.grid(row=row, column=col, columnspan=4, padx=x_sm_pad, sticky= tk.W + tk.E)

        # Action Buttons
        self.pick_next_pouch = ActionButton(frame, pub, 'PICK NEXT\nPOUCH', cmd_str='PickNextPouch', row=row+1, col=col+0)
        self.put_pouch_on_scale = ActionButton(frame, pub, 'PUT POUCH\nON SCALE', cmd_str='PutPouchOnScale', row=row+1, col=col+1)
        self.pick_pouch_from_scale = ActionButton(frame, pub, 'PICK POUCH\nFROM SCALE', cmd_str='PickPouchFromScale', row=row+1, col=col+2)
        self.pouch_on_strenght = ActionButton(frame, pub, 'POUCH ON\nSTRENGHT', cmd_str='PutPouchOnStrengthPlate', row=row+1, col=col+3)
        self.sheet_on_strenght = ActionButton(frame, pub, 'SHEET ON\nSTRENGHT', cmd_str='PutPaperSheetOnStrength', row=row+3, col=col+0)
        self.sheet_on_strenght = ActionButton(frame, pub, 'DISPOSE\nSHEET', cmd_str='DisposePaperSheetFromStrength', row=row+3, col=col+1)
        self.go_to_imaging = ActionButton(frame, pub, 'GO TO\nIMAGING', cmd_str='GoFromStrengthToImaging', row=row+3, col=col+2)
        self.start_test = ActionButton(frame, pub, 'START\nTEST', cmd_str='StartStrengthTest', row=row+3, col=col+3)

class GeneralSection: 
    def __init__(self, frame, pub, row, col):

        self.pub = pub
        
        # General Label
        self.label = tk.Label(frame, text = "General Controls", bg = bg_color, fg=title_color, pady=title_y_pad, font=title_font, anchor=CENTER)
        self.label.grid(row=row, column=col, columnspan=5, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Parameters
        self.robot_lab = tk.Label(frame, text = "Robot", bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.robot_lab.grid(row=row+1, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.plate_lab = tk.Label(frame, text = "Mark10 Plate", bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.plate_lab.grid(row=row+2, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.traj_lab = tk.Label(frame, text = "Trajectory",  bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.traj_lab.grid(row=row+3, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.queue_lab = tk.Label(frame, text = "Commands in Queue",  bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.queue_lab.grid(row=row+4, column=col, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Values
        self.robot = tk.Label(frame, text = "", bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.robot.grid(row=row+1, column=col+2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.plate = tk.Label(frame, text = "Not Moving", bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.plate.grid(row=row+2, column=col+2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.traj= tk.Label(frame, text = "Ready",  bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.traj.grid(row=row+3, column=col+2, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.queue= tk.Label(frame, text = "",  bg = bg_color, fg=title_color, pady=y_sm_pad, font=btn_font, anchor=CENTER)
        self.queue.grid(row=row+4, column=col+2, padx=x_lg_pad, sticky= tk.W + tk.E)

        # Buttons
        self.reset_btn = tk.Button(frame, text='RESET', bg=btn_bg_color, fg=btn_fg_color, pady=y_sm_pad, font=btn_font, anchor=CENTER, command=self.reset)
        self.reset_btn.grid(row=row+1, column=col+4, padx=x_lg_pad, sticky= tk.W + tk.E)

        self.rest_btn = tk.Button(frame, text='GO TO REST', bg=btn_bg_color, fg=btn_fg_color, pady=y_sm_pad, font=btn_font, anchor=CENTER, command=self.rest)
        self.rest_btn.grid(row=row+2, column=col+4, padx=x_lg_pad, pady=y_sm_pad, sticky= tk.W + tk.E)

        self.ping_btn = tk.Button(frame, text='PING', bg=btn_bg_color, fg=btn_fg_color, pady=y_sm_pad, font=btn_font, anchor=CENTER, command=self.ping)
        self.ping_btn.grid(row=row+3, column=col+4, padx=x_lg_pad, pady=y_sm_pad, sticky= tk.W + tk.E)

        self.stop_btn = tk.Button(frame, text='STOP TRAJECTORY', bg=btn_bg_color, fg='red', pady=y_sm_pad, font=btn_font, anchor=CENTER, command=self.stop)
        self.stop_btn.grid(row=row+4, column=col+4, padx=x_lg_pad, pady=y_sm_pad, sticky= tk.W + tk.E)

    def stop(self):
        self.pub.publish('StopCurrentRobotTrajectory')

    def rest(self):
        self.pub.publish('GoToRestPose')
    
    def ping(self):
        self.pub.publish('Ping')

    def reset(self):
        self.pub.publish('Reset')

    def change_robot(self, v):
        
        if v == 'moving': 
            self.robot.configure(text='MOVING', fg='green')
        elif v == 'idle': 
            self.robot.configure(text='IDLE', fg='yellow')
        else: 
            self.robot.configure(text=v.upper(), fg='red')
    
    def change_plate(self, status): 

        if status: 
            self.plate.configure(text='MOVING', fg='green')
        else: 
            self.plate.configure(text='NOT MOVING', fg='red')

    def change_traj(self, v): 
        self.traj.configure(text=v)
    
    def change_queue(self, v): 
        self.queue.configure(text=v)


class GUI:

    def __init__(self):
        
        
        # Colors
        global bg_color
        bg_color = '#111224'

        # Title'gripper_aperture_mm'
        font = 'Arial'
        global title_color
        title_color = 'white'

        global title_font
        title_font = (font, 20)

        global title_y_pad
        title_y_pad = 20

        global subtitle_fg_color
        subtitle_fg_color = 'white'

        global subtitle_font
        subtitle_font = (font, 14)

        global info_font
        info_font = (font, 16)

        global num_pouch_font
        num_pouch_font = (font, 30)

        # Buttons
        global btn_font
        btn_font = (font, 12)

        global btn_bg_color
        btn_bg_color = '#0F202A'

        global btn_fg_color
        btn_fg_color = '#00C2C6'

        global btn_fg_bright
        btn_fg_bright = 'white'

        # Padding
        global x_sm_pad
        global y_sm_pad
        global x_lg_pad
        global y_lg_pad

        x_sm_pad = 5
        y_sm_pad = 10
        x_lg_pad = 10
        y_lg_pad = 30

        row_space = 2

        
        self.root = tk.Tk()
        self.root.title('SUD GUI')

        # Root Configuration
        self.root.geometry("1500x800")
        self.root.configure(background=bg_color)

        # GUI Publisher
        gui_pub = rospy.Publisher('sud_posta_commands', String, queue_size=10)
        rospy.init_node('gui', anonymous=True)
        rospy.loginfo("GUI Node Started")

        # State Subscriber
        rospy.Subscriber('sud_posta_state', String, self.state_callback)

        # Error Subscriber
        rospy.Subscriber('sud_posta_errors', String, self.error_callback)

        # State memory
        self.last_state = {}

        # Logo
        logo_image = Image.open('./p&g_logo.png')
        resized_logo= logo_image.resize((60,30), Image.ANTIALIAS)
        pg_logo = ImageTk.PhotoImage(resized_logo)

        self.logo = Label(self.root, image = pg_logo, bg=bg_color)
        self.logo.image = pg_logo
        
        # Posta Client Status
        self.client_status = PostaClientStatus(self.root, row=0, col=0)

        # 1st Row
        self.drawers_section = DrawersSection(self.root, gui_pub, row=1, col=0)
        self.pouches_display = PouchesDisplay(self.root, row=1, col=7)
        self.time_disp = TimeDisplay(self.root, row=1, col=11)

        # Space Row
        self.root.rowconfigure(3, weight=row_space)
        
        # 2nd Row
        self.go_to_drawer_imaging_btn = GoToDrawerImaging(self.root, gui_pub, row=4, col=0)
        self.scale_disp = ScaleDisplay(self.root, gui_pub, row=4, col=7)

        # Space Row
        self.root.rowconfigure(6, weight=row_space)

        # 3rd Row      
        self.gripperlabel = tk.Label(self.root, text = "Gripper Status", bg = bg_color, fg='white', pady=title_y_pad, font=title_font)
        self.gripperlabel.grid(row=7, column=1, columnspan=4, padx=5, sticky= tk.W + tk.E)
        self.gripperbtn = GripperButton(self.root, gui_pub, row=8, col=1)
        self.pouch_in_gripper = PouchInGripper(self.root, row=8, col=2)
        self.gripper_aperture_setter = GripperApertureSetter(self.root, gui_pub, row=8, col=3)
        self.mark10_section = Mark10Section(self.root, gui_pub, row=7, col=7)

        # Space Row
        self.root.rowconfigure(10, weight=row_space)
        
        # 4th Row
        self.logo.grid(row=15, column=0)
        self.action_section = ActionSection(self.root, gui_pub, row=11, col=1)
        self.general_section = GeneralSection(self.root, gui_pub, row=11, col=7)
       
        self.root.mainloop()


    def state_callback(self, data):
        """ 
        Updates the state of the GUI everytime a new_state is received. 
        To update GUI state, we only consider value that changed from 
        previous state. 
        """
        state_string = data.data

        global new_state
        new_state = str_to_dict(state_string)

        # Difference between previous and new state
        sold = set(self.last_state.items())
        snew = set(new_state.items())

        # Parameters that changed and has to be updated
        to_update = dict(snew-sold)

        #rospy.loginfo(new_state['mark10_instrument_ready'])

        self.last_state = new_state
        
        for k, v in to_update.items():

            if k == 'timestamp': 
                day_time = strftime('%Y-%m-%d\n%H:%M:%S', localtime(v))
                self.time_disp.change_time(day_time)
                
            if k[:-2] == 'drawer':
                num = int(k.split('_')[1])
                self.drawers_section.open_close_drawer_num(num, v)

            elif k == 'gripper_aperture_mm':
                self.gripperbtn.button_callback(v)
                self.gripper_aperture_setter.change_value(v)

            elif k == 'number_of_detected_pouches':
                self.pouches_display.change_num_pouches(v)
            
            elif k == 'pouch_in_gripper':
                self.pouch_in_gripper.change_value(v)

            elif k == 'scale_weight':
                self.scale_disp.change_weight(v)
                scale_timestamp = to_update['scale_timestamp']
                day_time = strftime('%Y-%m-%d\n%H:%M:%S', localtime(scale_timestamp))
                self.scale_disp.change_time(day_time)
      
            elif k == 'posta_client_online': 
                self.client_status.change_status(v)

            elif k == 'mark10_instrument_ready':
                self.mark10_section.change_mark10_ready(v)
            
            elif k == 'mark10_test_completed':
                self.mark10_section.change_test_status(v)
            
            elif k == 'mark10_plate_distance':
                self.mark10_section.change_plate_distance(v)

            elif k == 'mark10_plate_moving_up': 
                self.general_section.change_plate(v)

            elif k == 'moveit_trajectory_status':
                self.general_section.change_traj(v)

            elif k == 'robot':
                self.general_section.change_robot(v)

            elif k == 'commands_in_queue':
                self.general_section.change_queue(v)

            elif k == 'error_msg':
                print(v)
                self.error_callback(v)

    def error_callback(self, error_msg):
        try:
            err_type, msg = error_msg.data.split('|')
        except:
            pass
        try:
            err_type, msg = error_msg.split('|')
        except:
            return

        if err_type == 'ERROR':
            showerror(title=err_type, message=msg)
        if err_type == 'WARNING':
            showwarning(title=err_type, message=msg)
        if err_type == 'INFO':
            showinfo(title=err_type, message=msg)


if __name__ == '__main__':
    try: 
        g = GUI()

    except rospy.ROSInterruptException: 
        pass
