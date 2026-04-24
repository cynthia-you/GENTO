import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import threading
import time
import random
import queue
import os
import glob
import math
import sys
from PythonSdk.MarvinRobot import MarvinRobot, RobotDataManager,error_dict, FXObjType, FXTerminalType
import ast
# from PIL import Image, ImageDraw, ImageTk
from pathlib import Path
import difflib
import re
import json
from typing import Optional

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("GentoPlatformV1004")
        self.root.geometry("1350x800")
        self.root.configure(bg="#f0f0f0")

        self.data_manager = None
        self.rt = None
        self.sg = None

        self.version = ''
        self.drag_mode = False

        self.params = []
        self.init_kd_variables()
        self.points1 = []
        self.points2 = []
        self.body_points = []
        self.head_points = []
        self.lift_points = []

        self.command1 = []
        self.command2 = []

        self.display_mode = 0
        self.mode_names = ["Position", "Velocity", "Torque", "TorqueEst", "CmdPosition"]
        self.data_keys = [('joint_pos'), ('joint_vel'), ('joint_torque'), ('joint_est_torque'), ('joint_cmd')]
        self.widgets = {}

        # Create control panel
        self.create_control_components()

        # Create main content area
        self.create_main_content()

        # Create component frames
        self.create_left_arm_components()
        self.create_separator()
        self.create_right_arm_components()
        self.create_separator()
        self.create_body_components()
        self.create_separator()
        self.create_head_components()
        self.create_separator()
        self.create_lift_components()

        # Create status bar
        self.create_status_bar()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.connected = False
        self.data_subscriber = None
        self.stop_event = threading.Event()
        self.thread = None

    def create_main_content(self):
        self.main_container = tk.Frame(self.root, bg="white", padx=5, pady=10)
        self.main_container.pack(fill="both", expand=True)
        self.main_canvas = tk.Canvas(self.main_container, bg="white", highlightthickness=0)
        self.main_scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.main_canvas.yview)
        self.scrollable_frame = tk.Frame(self.main_canvas, bg="white")
        self.stop_frame = tk.Frame(self.main_canvas, bg='white')
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        )
        self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.main_canvas.configure(yscrollcommand=self.main_scrollbar.set)
        self.main_canvas.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.main_scrollbar.pack(side="right", fill="y")
        self.main_canvas.bind_all("<MouseWheel>", self.on_mousewheel)

    def update_vertical_scrollbar(self, *args):
        self.v_scrollbar.set(*args)
        self.main_canvas.yview(*args)

    def update_horizontal_scrollbar(self, *args):
        self.h_scrollbar.set(*args)
        self.main_canvas.xview(*args)

    def scroll_horizontally(self, *args):
        self.main_canvas.xview(*args)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_shift_mousewheel(self, event):
        self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_horizontal_mousewheel(self, event):
        if event.delta:
            self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
            if event.num == 4:
                self.main_canvas.xview_scroll(-1, "units")
            elif event.num == 5:
                self.main_canvas.xview_scroll(1, "units")

    def create_separator(self):
        separator = tk.Frame(self.scrollable_frame, height=2, bg="#7F888C")
        separator.pack(fill="x", pady=(5,10))

    def create_left_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM0", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        # Status row
        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=12, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.left_state_main.pack(side="left")

        # Drag flag
        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_1.pack(side="left")

        # Low speed flag
        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_2.pack(side="left")

        # Error code
        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_3.pack(side="left")

        # Error detail (wraps to multiple lines, keep fill)
        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.left_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                       fg='#2c3e50', bg='white', pady=5,
                                       anchor='w', wraplength=120, justify='left')
        self.left_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_speed_entry.pack(side="left")
        self.left_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_accel_entry.pack(side="left")
        self.left_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn1 = tk.Button(param_row, text="Confirm Speed", width=15,
                               command=lambda: self.vel_acc_set('Arm0'),
                               bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn1.pack(side="left", padx=(0, 20))

        self.left_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                            command=lambda: self.show_impedance_dialog('Arm0'),
                                            bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.left_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button = tk.Button(state_row1, text="IDLE", width=10,
                                      command=lambda: self.idle_state('Arm0'),
                                      bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button = tk.Button(state_row1, text="Position", width=10,
                                         command=lambda :self.position_state('Arm0'), bg="#9fd4cf", fg="black",
                                         font=("Arial", 10, "bold"))
        self.position_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button = tk.Button(state_row1, text="JointImp", width=10,
                                         command=lambda :self.jointImp_state('Arm0'), bg="#d9d0ca", fg="black",
                                         font=("Arial", 10, "bold"))
        self.jointimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.cartimp_button = tk.Button(state_row2, text="CartImp", width=10,
                                        command=lambda :self.cartImp_state('Arm0'),
                                        bg="#dbd0db", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.forceimp_button = tk.Button(state_row2, text="ForceImp", width=10,
                                         command=lambda :self.forceImp_state('Arm0'), bg="#dfdcf2", fg="black",
                                         font=("Arial", 10, "bold"))
        self.forceimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Add drag mode selection (combobox left of button)
        drag_frame = tk.Frame(state_row2, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))
        self.drag_combo = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                       state="readonly", width=4)
        self.drag_combo.current(0)
        self.drag_combo.pack(side="left", padx=(0, 0))
        self.drag_btn = tk.Button(drag_frame, text="Drag", width=5,
                                  command=lambda: self.drag_state('Arm0'),
                                  bg="#FFA07A", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn.pack(side="left")


        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm0 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm0'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm0.pack(side="left", padx=(0, 5))

        self.get_servo_error_left_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                  command=lambda: self.error_get('Arm0'),
                                                  font=("Arial", 10, "bold"))
        self.get_servo_error_left_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_left_btn = tk.Button(control_frame, text="CR", width=5,
                                                 command=lambda: self.cr_state('Arm0'),
                                                 bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_left_btn.pack(side="left", padx=(0, 5))

        self.release_brake_left_btn = tk.Button(control_frame, text="Brake", width=10,
                                                command=lambda: self.brake('Arm0'),
                                                font=("Arial", 10, "bold"))
        self.release_brake_left_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_left_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                             command=lambda: self.release_brake('Arm0'),
                                             font=("Arial", 10, "bold"))
        self.hold_brake_left_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_pos_frame = tk.Frame(data_frame, bg="white")
        joint_pos_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_pos_frame, text="J1~J7:", font=('Arial', 10, 'bold'),width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.left_joint_text = tk.Text(joint_pos_frame, width=55, height=1,
                                       font=('Arial', 9), bg='white',
                                       relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.left_joint_text.tag_configure("center", justify='center')
        self.left_joint_text.pack(side="left")
        self.left_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        # # Cartesian positions row
        # cart_pos_frame = tk.Frame(data_frame, bg="white")
        # cart_pos_frame.pack(fill="x", pady=(0, 5))
        # tk.Label(cart_pos_frame, text="XYZABC:", font=('Arial', 10, 'bold'),width=8,
        #          bg='white').pack(side="left", padx=(0, 2))
        # self.left_cartesian_text = tk.Text(cart_pos_frame, width=55, height=1,
        #                                    font=('Arial', 9), bg='white',
        #                                    relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        # self.left_cartesian_text.tag_configure("center", justify='center')
        # self.left_cartesian_text.pack(side="left")
        # self.left_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        # self.left_cartesian_text.tag_add("center", "1.0", "end")
        # self.left_cartesian_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_add3 = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda :self.get_current_pos('Arm0'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        self.entry_var = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry = tk.Entry(row_cmd1, textvariable=self.entry_var, width=45)
        self.entry.pack(side="left", padx=(5, 5))

        self.btn_add1 = tk.Button(row_cmd1, text="Add", width=8, command=lambda :self.add_pos('Arm0'))
        self.btn_add1.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del1 = tk.Button(row_cmd2, text="Delete", width=8, command=lambda :self.delete_pos('Arm0'))
        self.btn_del1.pack(side="left", padx=(0, 5))

        self.combo1 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo1.pack(side="left", padx=(0, 5))

        self.btn_run1 = tk.Button(row_cmd2, text="Run", width=8, command=lambda:self.run_pos('Arm0'),
                                  font=("Arial", 11, "bold"), fg='white',
                                  bg='#EC2A23', border=5)
        self.btn_run1.pack(side="left", padx=(0, 5))

    def create_right_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM1", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                         fg='#34495e', bg='white', width=15, pady=3,
                                         relief=tk.SUNKEN, bd=1)
        self.right_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_1.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_2.pack(side="left")

        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_3.pack(side="left")

        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.right_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                        fg='#2c3e50', bg='white', pady=5,
                                        anchor='w', wraplength=120, justify='left')
        self.right_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_speed_entry.pack(side="left")
        self.right_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_accel_entry.pack(side="left")
        self.right_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15,
                              command=lambda: self.vel_acc_set('Arm1'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        self.right_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                             command=lambda: self.show_impedance_dialog('Arm1'),
                                             bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.right_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button_r = tk.Button(state_row1, text="IDLE", width=10,
                                        command=lambda: self.idle_state('Arm1'),
                                        bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button_r = tk.Button(state_row1, text="Position", width=10,
                                           command=lambda :self.position_state('Arm1'), bg="#9fd4cf", fg="black",
                                           font=("Arial", 10, "bold"))
        self.position_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button_r = tk.Button(state_row1, text="JointImp", width=10,
                                           command=lambda :self.jointImp_state('Arm1'), bg="#d9d0ca", fg="black",
                                           font=("Arial", 10, "bold"))
        self.jointimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.cartimp_button_r = tk.Button(state_row2, text="CartImp", width=10,
                                          command=lambda :self.cartImp_state('Arm1'),
                                          bg="#dbd0db", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.forceimp_button_r = tk.Button(state_row2, text="ForceImp", width=10,
                                           command=lambda :self.forceImp_state('Arm1'), bg="#dfdcf2", fg="black",
                                           font=("Arial", 10, "bold"))
        self.forceimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Add drag mode selection (combobox left of button)
        drag_frame = tk.Frame(state_row2, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))
        self.drag_combo_r = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                         state="readonly", width=4)
        self.drag_combo_r.current(0)
        self.drag_combo_r.pack(side="left", padx=(0, 0))
        self.drag_btn_r = tk.Button(drag_frame, text="Drag", width=5,
                                    command=lambda: self.drag_state('Arm1'),
                                    bg="#FFA07A", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn_r.pack(side="left")

        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm1 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm1'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm1.pack(side="left", padx=(0, 5))

        self.get_servo_error_right_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                   command=lambda: self.error_get('Arm1'),
                                                   font=("Arial", 10, "bold"))
        self.get_servo_error_right_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_right_btn = tk.Button(control_frame, text="CR", width=5,
                                                  command=lambda: self.cr_state('Arm1'),
                                                  bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_right_btn.pack(side="left", padx=(0, 5))

        self.release_brake_right_btn = tk.Button(control_frame, text="Brake", width=10,
                                                 command=lambda: self.brake('Arm1'),

                                                 font=("Arial", 10, "bold"))
        self.release_brake_right_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_right_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                              command=lambda: self.release_brake('Arm1'),
                                              font=("Arial", 10, "bold"))
        self.hold_brake_right_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_frame, text="J1~J7:", font=('Arial', 10, 'bold'),width=6,
                 bg='white').pack(side="left", padx=(0, 2))
        self.r_joint_text = tk.Text(joint_frame, width=55, height=1,
                                        font=('Arial', 9), bg='white',
                                        relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.r_joint_text.tag_configure("center", justify='center')
        self.r_joint_text.pack(side="left", fill="x")
        self.r_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        # # Cartesian positions row
        # cart_frame = tk.Frame(data_frame, bg="white")
        # cart_frame.pack(fill="x", pady=(0, 5))
        # tk.Label(cart_frame, text="XYZABC:", font=('Arial', 10, 'bold'),width=6,
        #          bg='white').pack(side="left", padx=(0, 2))
        # self.right_cartesian_text = tk.Text(cart_frame, width=55, height=1,
        #                                     font=('Arial', 9), bg='white',
        #                                     relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        # self.right_cartesian_text.tag_configure("center", justify='center')
        # self.right_cartesian_text.pack(side="left", fill="x")
        # self.right_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        # self.right_cartesian_text.tag_add("center", "1.0", "end")
        # self.right_cartesian_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_frame, text="J1~J7:", font=('Arial', 10, 'bold'),
                 bg='white').pack(side="left", padx=(0, 2))
        self.r_joint_text = tk.Text(joint_frame, width=55, height=1,
                                        font=('Arial', 9), bg='white',
                                        relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.r_joint_text.tag_configure("center", justify='center')
        self.r_joint_text.pack(side="left", fill="x", expand=True)
        self.r_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        # Cartesian positions row
        cart_frame = tk.Frame(data_frame, bg="white")
        cart_frame.pack(fill="x", pady=(0, 5))
        tk.Label(cart_frame, text="XYZABC (flange):", font=('Arial', 10, 'bold'),
                 bg='white').pack(side="left", padx=(0, 2))
        self.right_cartesian_text = tk.Text(cart_frame, width=55, height=1,
                                            font=('Arial', 9), bg='white',
                                            relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.right_cartesian_text.tag_configure("center", justify='center')
        self.right_cartesian_text.pack(side="left", fill="x", expand=True)
        self.right_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_cartesian_text.tag_add("center", "1.0", "end")
        self.right_cartesian_text.config(state="disabled")

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_get_cur_r = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda :self.get_current_pos('Arm1'))
        self.btn_get_cur_r.pack(side="left", padx=(0, 5))

        self.entry_var1 = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry1 = tk.Entry(row_cmd1, textvariable=self.entry_var1, width=45)
        self.entry1.pack(side="left", padx=(5, 5))

        self.btn_add_r = tk.Button(row_cmd1, text="Add", width=8, command=lambda :self.add_pos('Arm1'))
        self.btn_add_r.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del_r = tk.Button(row_cmd2, text="Delete", width=8, command=lambda :self.delete_pos('Arm1'))
        self.btn_del_r.pack(side="left", padx=(0, 5))

        self.combo2 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo2.pack(side="left", padx=(5, 5))

        self.btn_run_r = tk.Button(row_cmd2, text="Run", width=8, command=lambda :self.run_pos('Arm1'),
                                   font=("Arial", 11, "bold"), fg='white',
                                   bg='#EC2A23', border=5)
        self.btn_run_r.pack(side="left", padx=(0, 5))

    # ==================== Body Component ====================
    def create_body_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left side: status info
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="BODY", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0,5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.body_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=120, justify='left')
        self.body_error_detail.pack(fill="x", padx=5)

        # Middle area: parameters, status switching, error handling
        middle_frame = tk.Frame(content, bg="white")
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0,10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.body_speed_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.body_speed_entry.pack(side="left")
        self.body_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        tk.Label(param_row, text="Accel:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.body_accel_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.body_accel_entry.pack(side="left")
        self.body_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Body'), bg="#58C3EE", font=("Arial",9,"bold"))
        speed_btn.pack(side="left", padx=(0,20))
        self.body_impedance_btn = tk.Button(param_row, text="PD Params", width=15, state=tk.DISABLED, bg="#9C27B0", fg="white", font=("Arial",9,"bold"))
        self.body_impedance_btn.pack(side="left")

        # Status switching
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0,5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0,5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0,5))
        self.body_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Body'), bg="#2196F3", fg="white", font=("Arial",10,"bold"))
        self.body_idle_btn.pack(side="left", pady=(0,5), padx=(0,5))
        self.body_pos_btn = tk.Button(state_row1, text="Position", width=10, command=lambda :self.position_state('Body'), bg="#9fd4cf", fg="black", font=("Arial",10,"bold"))
        self.body_pos_btn.pack(side="left", pady=(0,5), padx=(0,5))
        self.body_jointimp_btn = tk.Button(state_row1, text="Torque", width=10, state=tk.DISABLED, bg="#d9d0ca", fg="black", font=("Arial",10,"bold"))
        self.body_jointimp_btn.pack(side="left", pady=(0,5), padx=(0,5))

        # Error handling
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0,10))
        self.body_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Body'), bg="#a0ebc8", fg="black", font=("Arial",10,"bold"), relief=tk.RAISED, bd=2)
        self.body_reset_btn.pack(side="left", padx=(0,5))
        self.body_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10, command=lambda: self.error_get('Body'), font=("Arial",10,"bold"))
        self.body_get_error_btn.pack(side="left", padx=(0,20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.body_brake_btn = tk.Button(control_frame, text="Brake", width=10, state=tk.DISABLED, font=("Arial",10,"bold"))
        self.body_brake_btn.pack(side="left", padx=(0,5))
        self.body_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, state=tk.DISABLED, font=("Arial",10,"bold"))
        self.body_unbrake_btn.pack(side="left")

        # Right side: realtime data and position command
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0,5))
        tk.Label(data_frame, text="Pos(J1~J6):", font=('Arial',10,'bold'), bg='white').pack(side="left", padx=(0,2))
        self.body_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial',9), bg='white', relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.body_pos_text.tag_configure("center", justify='center')
        self.body_pos_text.pack(side="left")
        self.body_pos_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # Position command
        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0,5))
        self.body_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda :self.get_current_pos('Body'))
        self.body_get_btn.pack(side="left", padx=(0,5))
        self.body_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.body_cmd_entry.insert(0, "0,0,0,0,0,0")
        self.body_cmd_entry.pack(side="left", padx=(5,5))
        self.body_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda :self.add_pos('Body'))
        self.body_add_btn.pack(side="left", padx=(20,5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0,5))
        self.body_del_btn = tk.Button(row_cmd2, text="Delete", width=8,command=lambda :self.delete_pos('Body'))
        self.body_del_btn.pack(side="left", padx=(0,5))
        self.body_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.body_combo.pack(side="left", padx=(0,5))
        self.body_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda :self.run_pos('Body'), font=("Arial",11,"bold"), fg='white', bg='#EC2A23', border=5)
        self.body_run_btn.pack(side="left", padx=(0,5))

        self.body_points = []
        self.body_combo['values'] = []

    # ==================== Head Component ====================
    def create_head_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0,5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0,10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0,10))
        tk.Label(status_title_frame, text="HEAD", font=('Arial',11,'bold'), fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0,5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.head_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=120, justify='left')
        self.head_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True,padx=(0,15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0,10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.head_speed_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.head_speed_entry.pack(side="left")
        self.head_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        tk.Label(param_row, text="Accel:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.head_accel_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.head_accel_entry.pack(side="left")
        self.head_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Head'), bg="#58C3EE", font=("Arial",9,"bold"))
        speed_btn.pack(side="left", padx=(0,20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0,5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0,5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0,5))
        self.head_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Head'), bg="#2196F3", fg="white", font=("Arial",10,"bold"))
        self.head_idle_btn.pack(side="left", pady=(0,5), padx=(0,5))
        self.head_pos_btn = tk.Button(state_row1, text="Position", width=10, command=lambda :self.position_state('Head'), bg="#9fd4cf", fg="black", font=("Arial",10,"bold"))
        self.head_pos_btn.pack(side="left", pady=(0,5), padx=(0,5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0,10))
        self.head_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Head'), bg="#a0ebc8", fg="black", font=("Arial",10,"bold"), relief=tk.RAISED, bd=2)
        self.head_reset_btn.pack(side="left", padx=(0,5))
        self.head_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10, command=lambda: self.error_get('Head'), font=("Arial",10,"bold"))
        self.head_get_error_btn.pack(side="left", padx=(0,20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.head_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=None, font=("Arial",10,"bold"))
        self.head_brake_btn.pack(side="left", padx=(0,5))
        self.head_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=None, font=("Arial",10,"bold"))
        self.head_unbrake_btn.pack(side="left")

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0,5))
        tk.Label(data_frame, text="Pos(J1~J3):", font=('Arial',10,'bold'), bg='white').pack(side="left", padx=(0,2))
        self.head_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial',9), bg='white', relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.head_pos_text.tag_configure("center", justify='center')
        self.head_pos_text.pack(side="left")
        self.head_pos_text.insert("1.0", "0.000,0.000,0.000")
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0,5))
        self.head_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda :self.get_current_pos('Head'))
        self.head_get_btn.pack(side="left", padx=(0,5))
        self.head_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.head_cmd_entry.insert(0, "0,0,0,0,0,0")
        self.head_cmd_entry.pack(side="left", padx=(5,5))
        self.head_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda :self.add_pos('Head'))
        self.head_add_btn.pack(side="left", padx=(20,5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0,5))
        self.head_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda :self.delete_pos('Head'))
        self.head_del_btn.pack(side="left", padx=(0,5))
        self.head_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.head_combo.pack(side="left", padx=(0,5))
        self.head_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda :self.run_pos('Head'), font=("Arial",11,"bold"), fg='white', bg='#EC2A23', border=5)
        self.head_run_btn.pack(side="left", padx=(0,5))

        self.head_points = []
        self.head_combo['values'] = []

    # ==================== Lift Component ====================
    def create_lift_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0,5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0,10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0,10))
        tk.Label(status_title_frame, text="LIFT", font=('Arial',11,'bold'), fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0,5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50',width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.lift_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=120, justify='left')
        self.lift_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0,10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.lift_speed_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.lift_speed_entry.pack(side="left")
        self.lift_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        tk.Label(param_row, text="Accel:", font=('Arial',9), bg='white').pack(side="left", padx=(0,2))
        self.lift_accel_entry = tk.Entry(param_row, width=5, font=('Arial',9), justify='center')
        self.lift_accel_entry.pack(side="left")
        self.lift_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial',9), bg='white').pack(side="left", padx=(0,5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Lift'), bg="#58C3EE", font=("Arial",9,"bold"))
        speed_btn.pack(side="left", padx=(0,20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0,5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0,5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0,5))
        self.lift_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Lift'), bg="#2196F3", fg="white", font=("Arial",10,"bold"))
        self.lift_idle_btn.pack(side="left", pady=(0,5), padx=(0,5))
        self.lift_pos_btn = tk.Button(state_row1, text="Position", width=10, command=lambda :self.position_state('Lift'), bg="#9fd4cf", fg="black", font=("Arial",10,"bold"))
        self.lift_pos_btn.pack(side="left", pady=(0,5), padx=(0,5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0,10))
        self.lift_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Lift'), bg="#a0ebc8", fg="black", font=("Arial",10,"bold"), relief=tk.RAISED, bd=2)
        self.lift_reset_btn.pack(side="left", padx=(0,5))
        self.lift_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10, command=lambda: self.error_get('Lift'), font=("Arial",10,"bold"))
        self.lift_get_error_btn.pack(side="left", padx=(0,20))

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="both", expand=True)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0,5))
        tk.Label(data_frame, text="Pos(J1):", font=('Arial',10,'bold'), bg='white').pack(side="left", padx=(0,2))
        self.lift_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial',9), bg='white', relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.lift_pos_text.tag_configure("center", justify='center')
        self.lift_pos_text.pack(side="left")
        self.lift_pos_text.insert("1.0", "0.000,0.000")
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0,5))
        self.lift_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda :self.get_current_pos('Lift'))
        self.lift_get_btn.pack(side="left", padx=(0,5))
        self.lift_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.lift_cmd_entry.insert(0, "0,0")
        self.lift_cmd_entry.pack(side="left", padx=(5,5))
        self.lift_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda :self.add_pos('Lift'))
        self.lift_add_btn.pack(side="left", padx=(20,5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0,5))
        self.lift_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda :self.delete_pos('Lift'))
        self.lift_del_btn.pack(side="left", padx=(0,5))
        self.lift_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.lift_combo.pack(side="left", padx=(0,5))
        self.lift_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda :self.run_pos('Lift'), font=("Arial",11,"bold"), fg='white', bg='#EC2A23', border=5)
        self.lift_run_btn.pack(side="left", padx=(0,5))

        self.lift_points = []
        self.lift_combo['values'] = []

    # ==================== Control Panel ====================
    def create_control_components(self):
        """Create the top control panel"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", pady=5)
        self.control_frame.pack(fill="x")

        # Connect button
        self.connect_btn = tk.Button(
            self.control_frame,
            text="Connect Robot",
            width=15,
            command=self.toggle_connection,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"))
        self.connect_btn.pack(side="left", padx=5)

        self.arm_ip_entry = tk.Entry(self.control_frame)
        self.arm_ip_entry.insert(0, "6,6,7,190")
        self.arm_ip_entry.pack(side="left", padx=5)

        # more func
        self.more_features_btn = tk.Button(
            self.control_frame,
            text="More Features",
            width=15,
            command=self.show_more_features,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.more_features_btn.pack(side="right", padx=5)

        # Estop
        self.mode_btn = tk.Button(
            self.control_frame,
            text="EmergencyStop",
            width=20,
            command=self.Estop,
            bg="#ebfa1e",
            fg="black",
            font=("Arial", 14, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # # End-effector passthrough
        # self.mode_btn = tk.Button(
        #     self.control_frame,
        #     text="CAN/485",
        #     width=15,
        #     command=self.eef_dialog,
        #     bg="#3BA4FD",
        #     fg="white",
        #     font=("Arial", 10, "bold"))
        # self.mode_btn.pack(side="right", padx=5)

        # Mode switch button
        self.mode_btn = tk.Button(
            self.control_frame,
            text="Position",
            width=15,
            command=self.toggle_display_mode,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # Status indicator
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=5)
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="disconnected", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")

    def create_status_bar(self):
        """Create the bottom status bar"""
        self.status_bar = tk.Frame(self.root, height=20)
        self.status_bar.pack(side="bottom", fill="x")
        self.version_label = tk.Label(
            self.status_bar, text=f"", fg="black", font=("Arial", 9))
        self.version_label.pack(side="left", padx=15)
        self.time_label = tk.Label(
            self.status_bar, text="", fg="black", font=("Arial", 9))
        self.time_label.pack(side="right", padx=15)
        self.update_time()

    # ==================== Core Control Methods (Adapted to new MarvinRobot API) ====================
    def toggle_connection(self):
        global_robot_ip = self.arm_ip_entry.get()
        if not self.connected:
            try:
                ip_parts = [int(x) for x in global_robot_ip.split(',')]
                if len(ip_parts) != 4:
                    raise ValueError
                version = robot.link(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3], log_switch=1)
                if version <= 0:
                    messagebox.showerror('Failed!', f"Robot connection failed, error code: {version}")
                    return
                self.version = str(version)

                if not robot.comm_clear(20):
                    messagebox.showerror('Failed!', "clear buffer failed")
                    return

                # Reset all objects
                for obj_type in [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1, FXObjType.OBJ_BODY,
                                 FXObjType.OBJ_HEAD, FXObjType.OBJ_LIFT]:
                    robot.reset_error(obj_type)

                robot.comm_send()
                time.sleep(0.5)

                self.connected = True
                self.connect_btn.config(text="Disconnect", bg="#F44336")
                self.status_label.config(text="Connected")
                self.status_light.config(fg="green")
                self.mode_btn.config(state="normal")

                self.data_manager = RobotDataManager(robot)
                self.update_data()
                self.update_ui()

            except Exception as e:
                messagebox.showerror('Error', f"Connection failed: {e}")
        else:
            self.connected = False
            if hasattr(self, 'data_manager') and self.data_manager:
                self.data_manager.stop()
                self.data_manager = None
            self.connect_btn.config(text="Connect Robot", bg="#4CAF50")
            self.status_label.config(text="Disconnected")
            self.status_light.config(fg="red")
            self.mode_btn.config(state="disabled")

    def update_data(self):
        if not self.connected :
            return
        if self.data_manager:
            self.rt = self.data_manager.latest_rt
            self.sg = self.data_manager.latest_sg
            self.update_ui()
        self.root.after(200, self.update_data)

    def update_ui(self):
        if self.rt is None or self.sg is None:
            return
        state_map = {
            0: "IDLE",
            1: "Position",
            2: "ImpJoint",
            3: "ImpCart",
            4: "ImpForce",
            5: "DragJoint",
            6: "DragCartX",
            7: "DragCartY",
            8: "DragCartZ",
            9: "DragCartR",
            10: "Release",
            11: "PD",
            100: "Error",
            101: "Transferring",
            200: "Unknown"
        }
        cur_state = robot.current_state(FXObjType.OBJ_ARM0)
        self.left_state_main.config(text=state_map[cur_state])

        if self.sg['arms'][0]['get']['tip_di']==1:
            self.left_state_1.config(text=f"Dragging")
        else:
            self.left_state_1.config(text=f"Drag off")
        if self.sg['arms'][0]['get']['low_speed_flag']==0:
            self.left_state_2.config(text=f"Moving")
        else:
            self.left_state_2.config(text=f"Stopped")
        arm_err = self.rt['arms'][0]['state']['err']
        self.left_state_3.config(text=f"{arm_err}")
        # Display error description using error_dict
        if arm_err != 0 and arm_err in error_dict:
            self.left_arm_error.config(text=f"Error {arm_err}: {error_dict[arm_err]}")
        else:
            self.left_arm_error.config(text="")

        key = self.data_keys[self.display_mode]
        if key == "CmdPosition":
            joint_pos_l = self.rt["arms"][0]["cmd"]["joint_cmd"]
        else:
            joint_pos_l = self.rt["arms"][0]["fb"][key]
        joint_text_l = ", ".join(f"{v:.3f}" for v in joint_pos_l)
        self.left_joint_text.config(state="normal")
        self.left_joint_text.delete("1.0", tk.END)
        self.left_joint_text.insert("1.0", joint_text_l)
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")


        # pose_matrix = robot.forward_kinematics_single(0, joint_pos_l)
        # if pose_matrix:
        #     xyzabc = robot.matrix2xyzabc(pose_matrix)
        #     cart_text_l = ", ".join(f"{v:.3f}" for v in xyzabc[:6])
        # else:
        #     cart_text_l = "0.000,0.000,0.000,0.000,0.000,0.000"
        # self.left_cartesian_text.config(state="normal")
        # self.left_cartesian_text.delete("1.0", tk.END)
        # self.left_cartesian_text.insert("1.0", cart_text_l)
        # self.left_cartesian_text.tag_add("center", "1.0", "end")
        # self.left_cartesian_text.config(state="disabled")

        # ==================== ARM1 ====================
        cur_state = robot.current_state(FXObjType.OBJ_ARM1)
        self.right_state_main.config(text=state_map[cur_state])

        if self.sg['arms'][1]['get']['tip_di']==1:
            self.right_state_1.config(text=f"Dragging")
        else:
            self.right_state_1.config(text=f"Drag off")

        if self.sg['arms'][1]['get']['low_speed_flag']==0:
            self.right_state_2.config(text=f"Moving")
        else:
            self.right_state_2.config(text=f"Stopped")
        arm_err_r = self.rt['arms'][1]['state']['err']
        self.right_state_3.config(text=f"{arm_err_r}")
        if arm_err_r != 0 and arm_err_r in error_dict:
            self.right_arm_error.config(text=f"Error {arm_err_r}: {error_dict[arm_err_r]}")
        else:
            self.right_arm_error.config(text="")

        if key == "CmdPosition":
            joint_pos_r = self.rt["arms"][1]["cmd"]["joint_cmd"]
        else:
            joint_pos_r = self.rt["arms"][1]["fb"][key]
        joint_text_r = ", ".join(f"{v:.3f}" for v in joint_pos_r)
        self.r_joint_text.config(state="normal")
        self.r_joint_text.delete("1.0", tk.END)
        self.r_joint_text.insert("1.0", joint_text_r)
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        # pose_matrix = robot.forward_kinematics_single(1, joint_pos_r)
        # if pose_matrix:
        #     xyzabc = robot.matrix2xyzabc(pose_matrix)
        #     cart_text_r = ", ".join(f"{v:.3f}" for v in xyzabc[:6])
        # else:
        #     cart_text_r = "0.000,0.000,0.000,0.000,0.000,0.000"
        # self.right_cartesian_text.config(state="normal")
        # self.right_cartesian_text.delete("1.0", tk.END)
        # self.right_cartesian_text.insert("1.0", cart_text_r)
        # self.right_cartesian_text.tag_add("center", "1.0", "end")
        # self.right_cartesian_text.config(state="disabled")



        # ==================== BODY ====================
        cur_state = robot.current_state(FXObjType.OBJ_BODY)
        self.body_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        body_err = self.rt['body']['state']['err']
        self.body_error_code.config(text=f"{body_err}")
        if body_err != 0 and body_err in error_dict:
            self.body_error_detail.config(text=f"Error {body_err}: {error_dict[body_err]}")
        else:
            self.body_error_detail.config(text="")
        body_pos = self.rt["body"]["fb_pos"]
        body_text = ", ".join(f"{v:.3f}" for v in body_pos)
        self.body_pos_text.config(state="normal")
        self.body_pos_text.delete("1.0", tk.END)
        self.body_pos_text.insert("1.0", body_text)
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # ==================== HEAD ====================
        cur_state = robot.current_state(FXObjType.OBJ_HEAD)
        self.head_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        head_err = self.rt['head']['state']['err']
        self.head_error_code.config(text=f"{head_err}")
        if head_err != 0 and head_err in error_dict:
            self.head_error_detail.config(text=f"Error {head_err}: {error_dict[head_err]}")
        else:
            self.head_error_detail.config(text="")
        head_pos = self.rt["head"]["fb_pos"]
        head_text = ", ".join(f"{v:.3f}" for v in head_pos)
        self.head_pos_text.config(state="normal")
        self.head_pos_text.delete("1.0", tk.END)
        self.head_pos_text.insert("1.0", head_text)
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        # ==================== LIFT ====================
        cur_state = robot.current_state(FXObjType.OBJ_LIFT)
        self.lift_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        lift_err = self.rt['lift']['state']['err']
        self.lift_error_code.config(text=f"{lift_err}")
        if lift_err != 0 and lift_err in error_dict:
            self.lift_error_detail.config(text=f"Error {lift_err}: {error_dict[lift_err]}")
        else:
            self.lift_error_detail.config(text="")
        lift_pos = self.rt["lift"]["fb_pos"]
        lift_text = ", ".join(f"{v:.3f}" for v in lift_pos)
        self.lift_pos_text.config(state="normal")
        self.lift_pos_text.delete("1.0", tk.END)
        self.lift_pos_text.insert("1.0", lift_text)
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

    def toggle_display_mode(self):
        self.display_mode = (self.display_mode + 1) % 5
        self.mode_btn.config(text=self.mode_names[self.display_mode])
        self.update_ui()

    def update_time(self):
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        self.version_label.config(text=f"controller version:{self.version}")
        self.root.after(1000, self.update_time)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_menu_click(self, item):
        print(f"Menu clicked: {item}")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application?"):
            self.root.destroy()

    def vel_acc_set(self, obj):
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return

            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM0, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM0, acc)
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM1, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM1, acc)
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_BODY, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_BODY, acc)
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_HEAD, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_HEAD, acc)
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_LIFT, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_LIFT, acc)
            else:
                raise ValueError(f"Unknown obj: {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def reset_error(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            robot.reset_error(obj_type)
        except Exception as e:
            messagebox.showerror('Error', f"Reset failed: {e}")

    def idle_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            robot.switch_to_idle(obj_type, 1000)
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def cr_state(self, obj):
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Invalid obj: {obj}')
            return
        try:
            arm_idx = 0 if obj == 'Arm0' else 1
            robot.switch_to_collab_release(arm_idx, 1000)
        except Exception as e:
            messagebox.showerror('Error', f"CR failed: {e}")

    def position_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            # Get current speed/acceleration (default 20 if not set)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
            else:
                vel = acc = 20
            robot.switch_to_position_mode(obj_type, 1000, vel, acc)
        except Exception as e:
            messagebox.showerror('Error', f"Set position state failed: {e}")

    def get_current_pos(self, obj):
        try:
            pose = None
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["joint_pos"]
                print(f"pose:{pose}")
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry.delete(0, tk.END)
                    self.entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["joint_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry1.delete(0, tk.END)
                    self.entry1.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
            elif obj == 'Body':
                pose = self.rt["body"]["fb_pos"]
                if pose and len(pose) == 6:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.body_cmd_entry.delete(0, tk.END)
                    self.body_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Body')
            elif obj == 'Head':
                pose = self.rt["head"]["fb_pos"]
                if pose and len(pose) == 3:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.head_cmd_entry.delete(0, tk.END)
                    self.head_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Head')
            elif obj == 'Lift':
                pose = self.rt["lift"]["fb_pos"]
                if pose and len(pose) == 2:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.lift_cmd_entry.delete(0, tk.END)
                    self.lift_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Lift')
            else:
                messagebox.showerror('Error', f'Unknown object: {obj}')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def add_pos(self, obj):
        if obj == 'Arm0':
            point_str = self.entry_var.get()
            num_points = 7
            points_list = self.points1
            combo = self.combo1
        elif obj == 'Arm1':
            point_str = self.entry1.get()
            num_points = 7
            points_list = self.points2
            combo = self.combo2
        elif obj == 'Body':
            point_str = self.body_cmd_entry.get()
            num_points = 6
            points_list = self.body_points
            combo = self.body_combo
        elif obj == 'Head':
            point_str = self.head_cmd_entry.get()
            num_points = 3
            points_list = self.head_points
            combo = self.head_combo
        elif obj == 'Lift':
            point_str = self.lift_cmd_entry.get()
            num_points = 2
            points_list = self.lift_points
            combo = self.lift_combo
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        is_valid, result = self.validate_point(point_str, num_points)
        if not is_valid:
            messagebox.showwarning("Wrong inputs", result)
            return

        if self.is_duplicate(result, points_list):
            messagebox.showwarning("Duplicate point", f"This point already exists in {obj} list")
            return

        points_list.insert(0, result)
        self.update_comboboxes()

    def delete_pos(self, obj):
        if obj == 'Arm0':
            combo = self.combo1
            points_list = self.points1
        elif obj == 'Arm1':
            combo = self.combo2
            points_list = self.points2
        elif obj == 'Body':
            combo = self.body_combo
            points_list = self.body_points
        elif obj == 'Head':
            combo = self.head_combo
            points_list = self.head_points
        elif obj == 'Lift':
            combo = self.lift_combo
            points_list = self.lift_points
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        selected_index = combo.current()
        if selected_index != -1 and selected_index < len(points_list):
            points_list.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", f"Please select a point to delete in {obj}")

    def update_comboboxes(self):
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2
        self.body_combo['values'] = self.body_points
        self.head_combo['values'] = self.head_points
        self.lift_combo['values'] = self.lift_points

        if self.points1:
            self.combo1.current(0)
        else:
            self.combo1.set('')

        if self.points2:
            self.combo2.current(0)
        else:
            self.combo2.set('')

        if self.body_points:
            self.body_combo.current(0)
        else:
            self.body_combo.set('')

        if self.head_points:
            self.head_combo.current(0)
        else:
            self.head_combo.set('')

        if self.lift_points:
            self.lift_combo.current(0)
        else:
            self.lift_combo.set('')

    def is_duplicate(self, point_list, target_list):
        point_tuple = tuple(point_list)
        for existing_point_str in target_list:
            try:
                existing_point = ast.literal_eval(existing_point_str)
                if tuple(existing_point) == point_tuple:
                    return True
            except:
                continue
        return False

    def run_pos(self, obj):
        try:
            selected = None
            point_list = []
            if obj == 'Arm0':
                selected = self.combo1.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM0, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm0")
            elif obj == 'Arm1':
                selected = self.combo2.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM1, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm1")
            elif obj == 'Body':
                selected = self.body_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 6)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success =  robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_BODY, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Body")
            elif obj == 'Head':
                selected = self.head_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 3)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success =  robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_HEAD, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Head")
            elif obj == 'Lift':
                selected = self.lift_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 2)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success =  robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_LIFT, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Lift")
            else:
                messagebox.showwarning("Warning", f"Unknown object: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def validate_point(self, point_str, nums):
        try:
            point_str = point_str.strip()
            if not point_str:
                return False, "The input cannot be empty."
            values = point_str.split(',')
            if len(values) != nums:
                return False, f"Please enter {nums} comma-separated numbers"
            validated_values = []
            for value in values:
                value = value.strip()
                if not value:
                    return False, "All positions must contain numbers and cannot be empty."
                if not value.isdigit():
                    try:
                        float(value)
                    except ValueError:
                        return False, f"'{value}' is not a valid number"
                validated_values.append(value)
            if len(validated_values) != nums:
                return False, f"The list length must be {nums}"
            normalized_str = ','.join(validated_values)
            return True, normalized_str
        except Exception as e:
            return False, f"Incorrect input format: {str(e)}"

    def eef_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("End-Effector Communication")
        drag_dialog.geometry("1300x400")
        drag_dialog.resizable(True, True)
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        parent = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        parent.pack(fill="both", expand=True)

        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="Arm0 send", command=lambda: self.send_data_eef('Arm0'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        self.com_text_1 = tk.Label(self.eef_frame_1, text="Channel", bg="white", width=5)
        self.com_text_1.grid(row=0, column=1, padx=5)

        self.com_select_combobox_1 = ttk.Combobox(
            self.eef_frame_1,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_1.current(0)
        self.com_select_combobox_1.grid(row=0, column=2, padx=5)

        self.com_entry_1 = tk.Entry(self.eef_frame_1, width=120)
        self.com_entry_1.insert(0, "01 00 00 00 FF FF FF FF FF FF FF FC")
        self.com_entry_1.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_delet_1 = tk.Button(self.eef_frame_1, text="Delete", command=lambda: self.delete_eef_command('Arm0'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly", width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="Arm0 receive", command=lambda: self.receive_data_eef('Arm0'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1 = tk.Button(self.eef_frame_1_2, text='Arm0 add', command=lambda: self.add_eef_command('Arm0'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2 = tk.Button(self.eef_frame_1_2, text='Arm1 add', command=lambda: self.add_eef_command('Arm1'))
        self.eef_add_2.grid(row=0, column=5, padx=5)

        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="Arm1 send", command=lambda: self.send_data_eef('Arm1'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        self.com_text_2 = tk.Label(self.eef_frame_2, text="Channel", bg="white", width=5)
        self.com_text_2.grid(row=0, column=1, padx=5)

        self.com_select_combobox_2 = ttk.Combobox(
            self.eef_frame_2,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_2.current(0)
        self.com_select_combobox_2.grid(row=0, column=2, padx=5)

        self.eef_delet_2 = tk.Button(self.eef_frame_2, text="Delete", command=lambda: self.delete_eef_command('Arm1'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly", width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="Arm1 receive", command=lambda: self.receive_data_eef('Arm1'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)

        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        recv_label1 = tk.Label(self.eef_frame_3, text="Arm0 received:")
        recv_label1.grid(row=0, column=0, padx=5)

        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm0 receive button,\nEnter data to send, click Arm0 send button,\nReceived end-effector data is refreshed at 1kHz')

        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        recv_label2 = tk.Label(self.eef_frame_3, text="Arm1 received:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm1 receive button,\nEnter data to send, click Arm1 send button,\nReceived end-effector data is refreshed at 1kHz')

        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def is_duplicate_command(self, point_list, target_list):
        for existing_point_str in target_list:
            if existing_point_str == point_list:
                return True
        return False

    def add_eef_command(self, obj):
        command_str = self.eef_entry.get()
        if obj == 'Arm0':
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in left arm list.")
                return
            else:
                self.command1.insert(0, command_str)
        elif obj == 'Arm1':
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in right arm list.")
                return
            else:
                self.command2.insert(0, command_str)
        self.update_combo_eef()

    def update_combo_eef(self):
        self.eef_combo1['values'] = self.command1
        self.eef_combo2['values'] = self.command2
        if self.command1:
            self.eef_combo1.current(0)
        else:
            self.eef_combo1.set('')
        if self.command2:
            self.eef_combo2.current(0)
        else:
            self.eef_combo2.set('')

    def send_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            com = 0
            com_str = ''
            sample_data = None
            if obj == 'Arm0':
                sample_data = self.eef_combo1.get()
                com_str = self.com_select_combobox_1.get()
                terminal = FXTerminalType.TERMINAL_ARM0
            elif obj == 'Arm1':
                sample_data = self.eef_combo2.get()
                com_str = self.com_select_combobox_2.get()
                terminal = FXTerminalType.TERMINAL_ARM1
            else:
                return

            if com_str == 'CAN':
                com = 1
            elif com_str == 'COM1':
                com = 2
            elif com_str == 'COM2':
                com = 3

            robot.terminal_clear(terminal)
            time.sleep(0.02)
            robot.terminal_set(terminal, com, sample_data)

            for _ in range(200):
                chn, data = robot.terminal_get(terminal)
                if chn > 0:
                    hex_str = data.hex().upper()
                    formatted_hex = ' '.join(hex_str[i:i + 2] for i in range(0, len(hex_str), 2))
                    if obj == 'Arm0':
                        self.recv_text1.delete('1.0', tk.END)
                        self.recv_text1.insert(tk.END, formatted_hex)
                    else:
                        self.recv_text2.delete('1.0', tk.END)
                        self.recv_text2.insert(tk.END, formatted_hex)
                    break
                time.sleep(0.005)
            else:
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, "No response")
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, "No response")
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def delete_eef_command(self, obj):
        if obj == 'Arm0':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")
        elif obj == 'Arm1':
            selected_index = self.eef_combo2.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")

    def receive_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
        try:
            terminal = FXTerminalType.TERMINAL_ARM0 if obj == 'Arm0' else FXTerminalType.TERMINAL_ARM1
            chn, data = robot.terminal_get(terminal)
            if chn > 0:
                text = data.decode(errors='replace')
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, text)
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, text)
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def show_more_features(self):
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="CAN/485", command=self.eef_dialog)
        menu.add_separator()
        # menu.add_command(label="Planning", command=self.planning_dialog)
        # menu.add_separator()
        menu.add_command(label="Sensors & Encoders", command=self.sensor_decoder_dialog)
        menu.add_separator()
        menu.add_command(label="FileClient", command=self.file_client_dialog)
        # menu.add_separator()
        # menu.add_command(label="Docs", )#command=self.open_doc()
        try:
            menu.tk_popup(
                self.more_features_btn.winfo_rootx(),
                self.more_features_btn.winfo_rooty() + self.more_features_btn.winfo_height()
            )
        finally:
            menu.grab_release()

    def sensor_decoder_dialog(self):
        button_w = 10
        sensor_encoder_window = tk.Toplevel(self.root)
        sensor_encoder_window.title("Sensor and encoder functions")
        sensor_encoder_window.geometry("800x400")
        sensor_encoder_window.configure(bg="white")
        sensor_encoder_window.transient(self.root)
        sensor_encoder_window.resizable(True, True)
        sensor_encoder_window.grab_set()

        self.sensor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_2.pack(fill="x",padx=5,pady=(15,10))
        self.sensor_main_tex = tk.Label(self.sensor_frame_2, text="Sensor offset reset", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_main_tex.pack(fill='x',padx=(5,20))

        self.sensor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_1.pack(fill="x")
        self.axis_text_ = tk.Label(self.sensor_frame_1, text="ARM0", bg="#D8F4F3")
        self.axis_text_.grid(row=0, column=0, padx=(5,5))
        #resetoffset
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm0'))
        self.get_offset_btn_1.grid(row=0, column=1, padx=5)

        self.axis_text__ = tk.Label(self.sensor_frame_1, text="ARM1", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=2, padx=(5,5))

        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm1'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=5)


        '''encoder'''
        self.encoder_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.encoder_frame_1.pack(fill="x",padx=5,pady=(25,10))
        self.encoder_frame_1 = tk.Label(self.encoder_frame_1, text="Motor encoder zeroing and error clearing", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.encoder_frame_1.pack(fill='x')
        '''left arm'''
        self.motor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_1.pack(fill="x")
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="Left   arm", bg="#D8F4F3")
        self.motor_text_1.grid(row=0, column=0, padx=(5,5))

        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('A'))
        self.motor_btn_1.grid(row=0, column=1, padx=5, pady=5)

        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder clearing error", bg="#7ED2B4",
                                     command=lambda: self.clear_motor_error('A'))
        self.motor_btn_3.grid(row=0, column=2, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")

        self.motor_text_1 = tk.Label(self.motor_frame_2, text="Right arm", bg="#F4E4D8")
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="Motor encoder zeroing",
                                      command=lambda: self.clear_motor_as_zero('B'))
        self.motor_btn_11.grid(row=0, column=1, padx=5)

        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="Encoder clearing error", bg="#7ED2B4",
                                      command=lambda: self.clear_motor_error('B'))
        self.motor_btn_31.grid(row=0, column=2, padx=5)

    def clear_sensor_offset(self, obj):
        if obj not in ('Arm0', 'Arm1'):
            raise ValueError(f"clear_sensor_offset only supports Arm0/Arm1, got {obj}")
        obj_type = self._obj_name_to_type(obj)
        return self.config_clear_sensor_offset(obj_type)

    def clear_motor_as_zero(self, arm):
        """Motor encoder zeroing (reset encoder offset) for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if arm == 'A':
                obj_type = FXObjType.OBJ_ARM0
            elif arm == 'B':
                obj_type = FXObjType.OBJ_ARM1
            else:
                raise ValueError("arm must be 'A' or 'B'")
            axis_mask = 0x7F  # All 7 axes
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            if not robot.config_reset_enc_offset(obj_type, axis_mask):
                messagebox.showerror('Failed!', f"Reset encoder offset failed for arm {arm}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def clear_motor_error(self, arm):
        """Clear encoder error for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if arm == 'A':
                obj_type = FXObjType.OBJ_ARM0
            elif arm == 'B':
                obj_type = FXObjType.OBJ_ARM1
            else:
                raise ValueError("arm must be 'A' or 'B'")
            axis_mask = 0x7F  # All 7 axes
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            if not robot.config_clear_enc_error(obj_type, axis_mask):
                messagebox.showerror('Failed!', f"Clear encoder error failed for arm {arm}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def file_client_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("File Transfer")
        dialog.geometry("300x150")
        dialog.configure(bg="white")
        dialog.transient(self.root)
        dialog.grab_set()

        send_btn = tk.Button(dialog, text="Send File to Robot", width=20,
                             command=self._send_file_to_robot, bg="#4CAF50", fg="white")
        send_btn.pack(pady=20)

        recv_btn = tk.Button(dialog, text="Receive File from Robot", width=20,
                             command=self._receive_file_from_robot, bg="#2196F3", fg="white")
        recv_btn.pack(pady=10)

    def _send_file_to_robot(self):
        local_path = filedialog.askopenfilename(title="Select file to send")
        if not local_path:
            return
        remote_path = simpledialog.askstring("Remote Path", "Enter remote path (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return
        if robot.send_file(local_path, remote_path):
            messagebox.showinfo("Success", f"File sent to {remote_path}")
        else:
            messagebox.showerror("Failed", "Send file failed")
        robot.comm_send()

    def _receive_file_from_robot(self):
        remote_path = simpledialog.askstring("Remote Path",
                                             "Enter remote path to receive (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        local_path = filedialog.asksaveasfilename(title="Save file as")
        if not local_path:
            return
        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return
        if robot.recv_file(local_path, remote_path):
            messagebox.showinfo("Success", f"File received and saved to {local_path}")
        else:
            messagebox.showerror("Failed", "Receive file failed")
        robot.comm_send()

    def planning_dialog(self):
        1
    def Estop(self):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
        try:
            robot.emergency_stop(0x1F)
        except Exception as e:
            messagebox.showerror('Error', f'Emergency stop failed: {e}')

    def show_impedance_dialog(self, obj):
        impedance_dialog = tk.Toplevel(self.root)
        impedance_dialog.title(f"Impedance parameter settings for {obj}")
        impedance_dialog.geometry("1200x500")
        impedance_dialog.configure(bg="white")
        impedance_dialog.transient(self.root)
        impedance_dialog.resizable(True,True)
        impedance_dialog.grab_set()

        if obj == 'Arm0':
            main_frame = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame.pack(fill="both", expand=True)
            title_label = tk.Label(
                main_frame,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label.pack(pady=(0, 10))

            params_frame = tk.Frame(main_frame, bg='white')
            params_frame.pack(fill="x", pady=(5, 10))
            joint_kd_a_button = tk.Button(params_frame, text="JointImp parameters", width=20,
                                          command=lambda: self.joint_kd_set('Arm0'))
            joint_kd_a_button.grid(row=0, column=0, padx=5,pady=10)
            k_a_label = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label.grid(row=0, column=1)
            k_a_entry = tk.Entry(params_frame, textvariable=self.k_a_entry,width=50)
            k_a_entry.grid(row=0, column=2, sticky="ew")
            d_a_label = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label.grid(row=0, column=3)
            d_a_entry = tk.Entry(params_frame,textvariable=self.d_a_entry, width=30)
            d_a_entry.grid(row=0, column=4)

            params_save_frame = tk.Frame(main_frame,bg='white')
            params_save_frame.pack(fill="x", pady=(0, 10))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters", command=lambda: self.save_param('Arm0'))
            save_param_a_button.pack(side='left',padx=(200,0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters", command=lambda: self.load_param('Arm0'))
            load_param_a_button.pack(side='left',padx=(100,10))

            cart_kd_a_button = tk.Button(params_frame, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm0'))
            cart_kd_a_button.grid(row=1, column=0, padx=5,pady=(20,10))
            k_a_label_ = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label_.grid(row=1, column=1)
            cart_k_a_entry = tk.Entry(params_frame, textvariable=self.cart_k_a_entry,width=50)
            cart_k_a_entry.grid(row=1, column=2, sticky="ew")
            d_a_label_ = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label_.grid(row=1, column=3)
            cart_d_a_entry = tk.Entry(params_frame,textvariable=self.cart_d_a_entry,width=30)
            cart_d_a_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame, bg='white')
            force_torque_frame.pack(fill="x", pady=(10, 5))
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):", font=('Arial', 9), bg='white').pack(side="left",
                                                                                                 padx=(0, 5))
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_a_entry, width=25)
            force_entry.pack(side="left", padx=(0, 20))
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9), bg='white').pack(side="left",
                                                                                                  padx=(0, 5))
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_a_entry, width=25)
            torque_entry.pack(side="left", padx=(0, 20))

            force_torque_btn = tk.Frame(main_frame, bg='white')
            force_torque_btn.pack(fill="x", padx=(200,0), pady=(0, 10))
            set_ft_btn = tk.Button(force_torque_btn, text="Set Force/Torque", width=15,
                                   command=lambda: self.force_torque_set('Arm0'),
                                   bg="#FFB6C1", font=("Arial", 9, "bold"))
            set_ft_btn.pack(side="left")

        elif obj == 'Arm1':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label1.pack(pady=(0, 10))

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="Set joint impedance parameters", width=20,
                                           command=lambda: self.joint_kd_set('Arm1'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.k_b_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.d_b_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            params_save_frame1 = tk.Frame(main_frame1, bg='white')
            params_save_frame1.pack(fill="x", pady=(0, 10))
            save_param_b_button = tk.Button(params_save_frame1, text="Save parameters",
                                            command=lambda: self.save_param('Arm1'))
            save_param_b_button.pack(side='left', padx=(200,0))
            load_param_b_button = tk.Button(params_save_frame1, text="Import parameters",
                                            command=lambda: self.load_param('Arm1'))
            load_param_b_button.pack(side='left', padx=(100,0))

            cart_kd_b_button = tk.Button(params_frame1, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm1'))
            cart_kd_b_button.grid(row=1, column=0, padx=5, pady=(20, 10))
            k_b_label_ = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_b_label_.grid(row=1, column=1)
            cart_k_b_entry = tk.Entry(params_frame1, textvariable=self.cart_k_b_entry, width=50)
            cart_k_b_entry.grid(row=1, column=2, sticky="ew")
            d_b_label_ = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label_.grid(row=1, column=3)
            cart_d_b_entry = tk.Entry(params_frame1, textvariable=self.cart_d_b_entry, width=30)
            cart_d_b_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame1, bg='white')
            force_torque_frame.pack(fill="x", pady=(10, 5))
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):",
                     font=('Arial', 9), bg='white').pack(side="left",
                                                         padx=(0, 5))
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_b_entry, width=25)
            force_entry.pack(side="left", padx=(0, 20))
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white').pack(side="left",
                                      padx=(0, 5))
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_b_entry, width=25)
            torque_entry.pack(side="left", padx=(0, 20))

            force_torque_btn = tk.Frame(main_frame1, bg='white')
            force_torque_btn.pack(fill="x", padx=(200, 0), pady=(0, 10))
            set_ft_btn = tk.Button(force_torque_btn, text="Set Force/Torque", width=15,
                                   command=lambda: self.force_torque_set('Arm1'),
                                   bg="#FFB6C1", font=("Arial", 9, "bold"))
            set_ft_btn.pack(side="left")

        elif obj == 'Body':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the PD parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label1.pack(pady=(0, 10))

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="PD parameters", width=20,
                                           command=lambda: self.joint_kd_set('Body'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='PDP:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.pdp_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='PDD:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.pdd_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            params_save_frame1 = tk.Frame(main_frame1, bg='white')
            params_save_frame1.pack(fill="x", pady=(0, 10))
            save_param_b_button = tk.Button(params_save_frame1, text="Save parameters",
                                            command=lambda: self.save_param('Body'))
            save_param_b_button.pack(side='left', padx=(200,0))
            load_param_b_button = tk.Button(params_save_frame1, text="Import parameters",
                                            command=lambda: self.load_param('Body'))
            load_param_b_button.pack(side='left',padx=(100,0))

    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="10000, 10000, 10000, 600, 600, 600, 20")
        self.cart_k_a_entry = tk.StringVar(value="10000, 10000, 10000, 600, 600, 600, 20")
        self.cart_d_a_entry = tk.StringVar(value="0.8, 0.8, 0.8, 0.2, 0.2, 0.2, 1")
        self.cart_d_b_entry = tk.StringVar(value="0.8, 0.8, 0.8, 0.2, 0.2, 0.2, 1")

        self.k_a_entry = tk.StringVar(value="12, 12, 12, 10, 9, 9, 7")
        self.k_b_entry = tk.StringVar(value="12, 12, 12, 10, 9, 9, 7")
        self.d_a_entry = tk.StringVar(value="0.5,0.5,0.5,0.5,0.5,0.5,0.5")
        self.d_b_entry = tk.StringVar(value="0.5,0.5,0.5,0.5,0.5,0.5,0.5")

        self.force_a_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_a_entry = tk.StringVar(value="0,1,0,5,10")
        self.force_b_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_b_entry = tk.StringVar(value="0,1,0,5,10")

        self.pdp_entry = tk.StringVar(value="12, 12, 12, 10, 9, 9, 7")
        self.pdd_entry = tk.StringVar(value="0.5,0.5,0.5,0.5,0.5,0.5,0.5")

    def joint_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            k_var = self.k_a_entry
            d_var = self.d_a_entry
            obj_type = FXObjType.OBJ_ARM0
            num_joints = 7
        elif obj == 'Arm1':
            k_var = self.k_b_entry
            d_var = self.d_b_entry
            obj_type = FXObjType.OBJ_ARM1
            num_joints = 7
        elif obj == 'Body':
            k_var = self.pdp_entry
            d_var = self.pdd_entry
            obj_type = FXObjType.OBJ_BODY
            num_joints = 6
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K/PDP parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K/PDP format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D/PDD parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D/PDD format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return

        if obj in ('Arm0', 'Arm1'):
            success = robot.runtime_set_joint_k(obj_type, k_list) and robot.runtime_set_joint_d(obj_type, d_list)
        else:  # Body
            success = robot.runtime_set_body_pdp(k_list) and robot.runtime_set_body_pdd(d_list)

        if not success:
            messagebox.showerror('Failed!', f"Set {obj} parameters failed")
        robot.comm_send()

    def cart_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Cartesian impedance not supported for {obj}')
            return

        obj_type = FXObjType.OBJ_ARM0 if obj == 'Arm0' else FXObjType.OBJ_ARM1
        k_var = self.cart_k_a_entry if obj == 'Arm0' else self.cart_k_b_entry
        d_var = self.cart_d_a_entry if obj == 'Arm0' else self.cart_d_b_entry

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "Cartesian K parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, 7)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid Cartesian K format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "Cartesian D parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, 7)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid Cartesian D format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return
        success = robot.runtime_set_cart_k(obj_type, k_list) and robot.runtime_set_cart_d(obj_type, d_list)
        if not success:
            messagebox.showerror('Failed!', f"Set {obj} Cartesian parameters failed")
        robot.comm_send()

    def save_param(self, obj):
        self.params = []
        if obj == 'Arm0':
            params_to_save = [
                self.k_a_entry.get(),
                self.d_a_entry.get(),
                self.cart_k_a_entry.get(),
                self.cart_d_a_entry.get()
            ]
        elif obj == 'Arm1':
            params_to_save = [
                self.k_b_entry.get(),
                self.d_b_entry.get(),
                self.cart_k_b_entry.get(),
                self.cart_d_b_entry.get()
            ]
        elif obj == 'Body':
            params_to_save = [
                self.pdp_entry.get(),
                self.pdd_entry.get()
            ]
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Save {obj} parameters"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'w') as f:
                for param in params_to_save:
                    f.write(param.strip() + '\n')
            messagebox.showinfo("Success", f"{obj} parameters saved successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_param(self, obj):
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Select parameter file for {obj}"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'r') as f:
                lines = [line.strip() for line in f.readlines() if line.strip()]
        except Exception as e:
            messagebox.showerror("Error", f"Error reading file: {str(e)}")
            return

        if obj == 'Arm0':
            if len(lines) >= 4:
                self.k_a_entry.set(lines[0])
                self.d_a_entry.set(lines[1])
                self.cart_k_a_entry.set(lines[2])
                self.cart_d_a_entry.set(lines[3])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
        elif obj == 'Arm1':
            if len(lines) >= 4:
                self.k_b_entry.set(lines[0])
                self.d_b_entry.set(lines[1])
                self.cart_k_b_entry.set(lines[2])
                self.cart_d_b_entry.set(lines[3])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
        elif obj == 'Body':
            if len(lines) >= 2:
                self.pdp_entry.set(lines[0])
                self.pdd_entry.set(lines[1])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 2)")
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")

    def force_torque_set(self, obj):
        """Set force and torque control parameters for Arm0/Arm1."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Force/Torque not supported for {obj}')
            return
        try:
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM0
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM1

            # Parse force list (5 floats)
            force_list = [float(x) for x in force_str.split(',')] if force_str else [0] * 5
            if len(force_list) != 5:
                messagebox.showerror('Error', 'Force Ctrl must have 5 comma-separated values')
                return
            torque_list = [float(x) for x in torque_str.split(',')] if torque_str else [0] * 5
            if len(torque_list) != 5:
                messagebox.showerror('Error', 'Torque Ctrl must have 5 comma-separated values')
                return

            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            success = robot.runtime_set_force_ctrl(obj_type, force_list) and \
                      robot.runtime_set_torque_ctrl(obj_type, torque_list)
            if not success:
                messagebox.showerror('Failed!', f"Set force/torque failed for {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Force/Torque set failed: {e}")

    def _obj_name_to_type(self, name):
        mapping = {
            'Arm0': FXObjType.OBJ_ARM0,
            'Arm1': FXObjType.OBJ_ARM1,
            'Body': FXObjType.OBJ_BODY,
            'Head': FXObjType.OBJ_HEAD,
            'Lift': FXObjType.OBJ_LIFT
        }
        return mapping.get(name, FXObjType.OBJ_ARM0)

    def jointImp_state(self, obj):
        """Switch to Joint Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Joint Impedance mode')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            # Get current speed/acceleration from the corresponding arm's entry
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.k_a_entry.get().strip()
                d_str = self.d_a_entry.get().strip()
            else:  # Arm1
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.k_b_entry.get().strip()
                d_str = self.d_b_entry.get().strip()
            # Parse K and D lists
            k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
            d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
            if len(k_list) != 7 or len(d_list) != 7:
                messagebox.showerror('Error', 'K/D must have 7 values')
                return
            ret = robot.switch_to_imp_joint_mode(obj_type, 1000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Joint Impedance failed for {obj}')
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f'Joint Impedance switch failed: {e}')

    def cartImp_state(self, obj):
        """Switch to Cartesian Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Cartesian Impedance mode')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.cart_k_a_entry.get().strip()
                d_str = self.cart_d_a_entry.get().strip()
            else:  # Arm1
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.cart_k_b_entry.get().strip()
                d_str = self.cart_d_b_entry.get().strip()
            k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
            d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
            if len(k_list) != 7 or len(d_list) != 7:
                messagebox.showerror('Error', 'Cartesian K/D must have 7 values')
                return
            ret = robot.switch_to_imp_cart_mode(obj_type, 1000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Cartesian Impedance failed for {obj}')
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f'Cartesian Impedance switch failed: {e}')

    def forceImp_state(self, obj):
        """Switch to Force Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Force Impedance mode')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
            # Parse to list of floats
            force_ctrl = [float(x) for x in force_str.split(',')] if force_str else [0.0] * 5
            torque_ctrl = [float(x) for x in torque_str.split(',')] if torque_str else [0.0] * 5
            if len(force_ctrl) != 5 or len(torque_ctrl) != 5:
                messagebox.showerror('Error', 'Force Ctrl and Torque Ctrl must each have 5 comma-separated values')
                return
            ret = robot.switch_to_imp_force_mode(obj_type, 1000, force_ctrl, torque_ctrl)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Force Impedance failed for {obj}')
            robot.comm_send()
        except ValueError as e:
            messagebox.showerror('Error', f'Invalid number format in Force/Torque: {e}')
        except Exception as e:
            messagebox.showerror('Error', f'Force Impedance switch failed: {e}')

    def drag_state(self, obj):
        """Switch to drag mode based on combo selection."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support drag mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            # Get selected drag type from the corresponding combo box
            if obj == 'Arm0':
                mode = self.drag_combo.get()
            else:
                mode = self.drag_combo_r.get()
            if mode == "joint":
                ret = robot.switch_to_drag_joint(obj_type, 1000)
            elif mode == "cartX":
                ret = robot.switch_to_drag_cart_x(obj_type, 1000)
            elif mode == "cartY":
                ret = robot.switch_to_drag_cart_y(obj_type, 1000)
            elif mode == "cartZ":
                ret = robot.switch_to_drag_cart_z(obj_type, 1000)
            elif mode == "cartR":
                ret = robot.switch_to_drag_cart_r(obj_type, 1000)
            else:
                messagebox.showerror('Error', f'Unknown drag mode: {mode}')
                return
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to {mode} drag failed for {obj}')
        except Exception as e:
            messagebox.showerror('Error', f'Drag mode switch failed: {e}')

    def error_get(self, obj):
        """Get servo error codes for the specified object and display in hex format."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            msg = robot.get_servo_error_codes(obj_type)
            if msg is None:
                messagebox.showwarning("Warning", f"Failed to get error codes for {obj}")
            else:
                messagebox.showinfo(f'{obj} Servo Error Details', msg)
        except Exception as e:
            messagebox.showerror('Error', f"Failed to get error codes: {e}")

    def release_brake(self, obj):
        """Release brake for the specified object (unlock)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            # Unlock brakes for all axes (axis_mask = 0xFF for up to 8 axes)
            # Adjust axis count based on object type
            if obj in ('Arm0', 'Arm1'):
                axis_mask = 0x7F  # 7 axes
            elif obj == 'Body':
                axis_mask = 0x3F  # 6 axes
            elif obj == 'Head':
                axis_mask = 0x07  # 3 axes
            elif obj == 'Lift':
                axis_mask = 0x03  # 2 axes
            else:
                axis_mask = 0xFF
            success = robot.config_brake_unlock(obj_type, axis_mask)
            if not success:
                messagebox.showerror('Failed!', f"Release brake failed for {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Release brake failed: {e}")

    def brake(self, obj):
        """Apply brake (lock) for the specified object."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            # Lock brakes for all axes
            if obj in ('Arm0', 'Arm1'):
                axis_mask = 0x7F
            elif obj == 'Body':
                axis_mask = 0x3F
            elif obj == 'Head':
                axis_mask = 0x07
            elif obj == 'Lift':
                axis_mask = 0x03
            else:
                axis_mask = 0xFF
            success = robot.config_brake_lock(obj_type, axis_mask)
            if not success:
                messagebox.showerror('Failed!', f"Brake lock failed for {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Brake lock failed: {e}")

if __name__ == "__main__":
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 130
    data_queue = queue.Queue()
    crr_pth = os.getcwd()
    robot = MarvinRobot()

    # '''ini kine of ARM0 & ARM1'''
    # arm0_kine=robot.init_single_arm_kinematics(0,"ccs_m6_40.MvKDCfg")
    # arm1_kine =robot.init_single_arm_kinematics(1,"ccs_m6_40.MvKDCfg")
    #
    # robot.kinematics_log_switch(arm0_kine,0)
    # robot.kinematics_log_switch(arm1_kine, 0)

    root = tk.Tk()
    style = ttk.Style()
    style.configure(
        "MyCustom.TLabelframe",
        font=("Arial", 12, "italic"),
        foreground="darkblue",
        background="white"
    )
    app = App(root)
    root.mainloop()