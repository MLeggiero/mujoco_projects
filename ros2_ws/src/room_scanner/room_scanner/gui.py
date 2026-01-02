#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from typing import Optional
import tkinter as tk
from tkinter import ttk
import threading

class InternalScannerGUI(Node):
    """ROS 2 Node for the Scanner GUI"""
    def __init__(self):
        super().__init__('scanner_gui')
        
        # Clients for RTAB-Map services
        self.resume_client = self.create_client(Empty, '/rtabmap/resume')
        self.pause_client = self.create_client(Empty, '/rtabmap/pause')
        self.save_map_client = self.create_client(Empty, '/rtabmap/save_map')
        self.reset_client = self.create_client(Empty, '/rtabmap/reset')
        
        self.get_logger().info('Scanner GUI Node Initialized')

    def call_service_async(self, client, service_name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {service_name} not available')
            return False
            
        future = client.call_async(Empty.Request())
        # We don't wait for result here to avoid blocking GUI
        return True

class App:
    """Tkinter Application"""
    def __init__(self, root, ros_node: InternalScannerGUI):
        self.root = root
        self.node = ros_node
        self.root.title("Room Scanner Control")
        self.root.geometry("300x400")
        self.root.resizable(False, False)
        
        # Style
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Helvetica', 12), padding=10)
        self.style.configure('Header.TLabel', font=('Helvetica', 14, 'bold'))
        self.style.configure('Status.TLabel', font=('Helvetica', 10))

        # Header
        header = ttk.Label(root, text="Room Scanner", style='Header.TLabel')
        header.pack(pady=20)

        # Status
        self.status_var = tk.StringVar(value="Status: Ready")
        status_label = ttk.Label(root, textvariable=self.status_var, style='Status.TLabel')
        status_label.pack(pady=10)

        # Buttons Frame
        frame = ttk.Frame(root, padding=20)
        frame.pack(fill=tk.BOTH, expand=True)

        # Start/Resume
        self.btn_start = ttk.Button(frame, text="▶ Resume Scanning", command=self.on_resume)
        self.btn_start.pack(fill=tk.X, pady=5)

        # Pause
        self.btn_pause = ttk.Button(frame, text="⏸ Pause Scanning", command=self.on_pause)
        self.btn_pause.pack(fill=tk.X, pady=5)

        # Save
        self.btn_save = ttk.Button(frame, text="💾 Save Map", command=self.on_save)
        self.btn_save.pack(fill=tk.X, pady=20)

        # Reset
        self.btn_reset = ttk.Button(frame, text="🔄 Reset Mapping", command=self.on_reset)
        self.btn_reset.pack(fill=tk.X, pady=5)

        # Help Text
        help_text = ttk.Label(frame, text="Use 'Resume' to start map updates.\nUse 'Pause' to freeze map.\n'Save Map' saves to database.", justify=tk.CENTER, foreground="gray")
        help_text.pack(pady=20)

    def update_status(self, text, color="black"):
        self.status_var.set(f"Status: {text}")
        
    def on_resume(self):
        self.update_status("Resuming...", "blue")
        if self.node.call_service_async(self.node.resume_client, 'Resume'):
            self.update_status("Scanning", "green")
        else:
            self.update_status("Error: Service Unavailable", "red")

    def on_pause(self):
        self.update_status("Pausing...", "blue")
        if self.node.call_service_async(self.node.pause_client, 'Pause'):
            self.update_status("Paused", "orange")
        else:
            self.update_status("Error: Service Unavailable", "red")

    def on_save(self):
        self.update_status("Saving Map...", "blue")
        if self.node.call_service_async(self.node.save_map_client, 'Save Map'):
            self.update_status("Map Saved request sent", "green")
            # Revert status after 2 seconds
            self.root.after(2000, lambda: self.update_status("Scanning" if "Scanning" in self.status_var.get() else "Paused"))
        else:
            self.update_status("Error: Service Unavailable", "red")

    def on_reset(self):
        if tk.messagebox.askyesno("Confirm Reset", "Are you sure you want to clear the current map?"):
            self.update_status("Resetting...", "blue")
            if self.node.call_service_async(self.node.reset_client, 'Reset'):
                self.update_status("Map Reset", "black")
                self.root.after(2000, lambda: self.update_status("Ready"))
            else:
                self.update_status("Error: Service Unavailable", "red")

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    
    gui_node = InternalScannerGUI()
    
    # Spin ROS in a separate thread so GUI doesn't freeze
    spin_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
    spin_thread.start()
    
    # Start GUI
    root = tk.Tk()
    app = App(root, gui_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
