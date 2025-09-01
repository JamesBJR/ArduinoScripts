# stepper_gui_plus.py
# Enhanced Tkinter GUI for Arduino stepper control
# - Steps/Rev (default 200 here)
# - Speed mode: RPM or Steps/s (converts to V <steps/s>)
# - Presets: 1/4, 1, 2, 5 rev; Jog: ±10/±100/±1000 steps
# Requires: pyserial  (pip install pyserial)

import sys, threading, time, queue
import serial, serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox

APP_TITLE = "Seal Stepper Controller GUI"
BAUD = 115200

def clamp(v, lo, hi):
    try:
        v = float(v)
    except Exception:
        return lo
    return max(lo, min(hi, v))



class SerialManager:
    def __init__(self):
        self.ser = None
        self.read_q = queue.Queue()
        self._stop = threading.Event()
        self.reader_thread = None

    def open(self, port, baud=BAUD, timeout=0.02):
        self.close()
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            self._stop.clear()
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            return True, ""
        except Exception as e:
            self.ser = None
            return False, str(e)

    def close(self):
        self._stop.set()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def write_line(self, line: str):
        if not self.is_open():
            raise RuntimeError("Serial not connected")
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode("utf-8", errors="ignore"))

    def _reader_loop(self):
        buf = b""
        while not self._stop.is_set() and self.ser and self.ser.is_open:
            try:
                b = self.ser.read(256)
                if b:
                    buf += b
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            self.read_q.put(line.decode("utf-8", errors="ignore"))
                        except Exception:
                            pass
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

class StepperApp(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=10)
        self.master = master
        self.ser = SerialManager()
        self.log = None  # Initialize log to None for safety

        self.grid(sticky="nsew")
        self.master.title(APP_TITLE)
        self.master.minsize(850, 520)
        # UI state
        self.port_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Disconnected")
        # Session variables used by controls (ensure they exist before callbacks)
        self.steps_rev_var = tk.StringVar(value="400")
        self.speed_mode_var = tk.StringVar(value="RPM")
        self.speed_val_var = tk.StringVar(value="300.0")
        self.accel_val_var = tk.StringVar(value="150.0")
        self.steps_var = tk.StringVar(value="400")
        self.motor_position = 0
        self.dir_var = tk.StringVar(value="+")

        # Build UI
        self._build_ui()

        # Polling and auto-connect
        self.after(100, self._refresh_ports)
        self.after(500, self._try_auto_connect_arduino)

        # Serial read polling
        self._poll_serial()
    # UI construction
    def _build_ui(self):
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)

        # Connection
        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", pady=(0,8))
        conn.columnconfigure(7, weight=1)

        ttk.Label(conn, text="Port").grid(row=0, column=0, padx=(8,4), pady=6, sticky="w")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=26, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4, pady=6, sticky="w")
        ttk.Button(conn, text="↻", width=3, command=self._refresh_ports).grid(row=0, column=2, padx=(0,8))
        ttk.Button(conn, text="Connect", command=self._connect).grid(row=0, column=3, padx=4)
        ttk.Button(conn, text="Disconnect", command=self._disconnect).grid(row=0, column=4, padx=4)
    # Removed Auto-connect checkbox; auto-connect is handled programmatically
        ttk.Label(conn, textvariable=self.status_var).grid(row=0, column=6, padx=8, sticky="e")

        # Sensor state checkbox
        self.pin5_state_var = tk.BooleanVar(value=False)
        self.pin5_checkbox = ttk.Checkbutton(conn, text="Sensor (Pin 5)", variable=self.pin5_state_var, state="disabled")
        self.pin5_checkbox.grid(row=0, column=7, padx=8)

        # Expected motor position label
        self.expected_position_var = tk.StringVar(value="0")
        ttk.Label(conn, text="Expected Motor Position:").grid(row=1, column=0, padx=(8,4), pady=6, sticky="w")
        self.expected_position_label = ttk.Label(conn, textvariable=self.expected_position_var)
        self.expected_position_label.grid(row=1, column=1, padx=4, pady=6, sticky="w")

        # True encoder position label
        self.encoder_position_var = tk.StringVar(value="0")
        ttk.Label(conn, text="True Encoder Position:").grid(row=1, column=2, padx=(8,4), pady=6, sticky="w")
        self.encoder_position_label = ttk.Label(conn, textvariable=self.encoder_position_var)
        self.encoder_position_label.grid(row=1, column=3, padx=4, pady=6, sticky="w")

        # Converted encoder position label
        self.converted_encoder_position_var = tk.StringVar(value="0.00")
        ttk.Label(conn, text="Converted Encoder Position:").grid(row=1, column=4, padx=(8,4), pady=6, sticky="w")
        self.converted_encoder_position_label = ttk.Label(conn, textvariable=self.converted_encoder_position_var)
        self.converted_encoder_position_label.grid(row=1, column=5, padx=4, pady=6, sticky="w")

        # Settings
        cfg = ttk.LabelFrame(self, text="Settings")
        cfg.grid(row=1, column=0, sticky="ew", pady=(0,8))
        for c in range(12):
            cfg.columnconfigure(c, weight=1)

        ttk.Label(cfg, text="Speed:").grid(row=0, column=0, padx=4, pady=6, sticky="e")
        speed_entry = ttk.Entry(cfg, textvariable=self.speed_val_var, width=10)
        speed_entry.grid(row=0, column=1, padx=(0,8), sticky="w")
        ttk.Radiobutton(cfg, text="RPM", variable=self.speed_mode_var, value="RPM").grid(row=0, column=2, padx=4, sticky="w")
        ttk.Radiobutton(cfg, text="SPS", variable=self.speed_mode_var, value="SPS").grid(row=0, column=3, padx=4, sticky="w")
        ttk.Button(cfg, text="Set Speed", command=self._set_speed).grid(row=0, column=4, padx=4, sticky="w")

        # Controls
        ctrls = ttk.LabelFrame(self, text="Controls")
        ctrls.grid(row=2, column=0, sticky="ew", pady=(0,8))
        for c in range(14):
            ctrls.columnconfigure(c, weight=1)

        ttk.Button(ctrls, text="Enable (E)", command=lambda:self._send("E")).grid(row=0, column=0, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Disable (D)", command=lambda:self._send("D")).grid(row=0, column=1, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Soft Stop (Z)", command=lambda:self._send("Z")).grid(row=0, column=2, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="+1 Rev", command=lambda:self._move_frac(1.00, True)).grid(row=0, column=3, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="-1 Rev", command=lambda:self._move_frac(1.00, False)).grid(row=0, column=4, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Stay On", command=self._spin_motor).grid(row=0, column=5, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Stop", command=self._stop_motor).grid(row=0, column=6, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Home", command=self._home_motor).grid(row=0, column=7, padx=4, pady=6, sticky="ew")
        ttk.Button(ctrls, text="Read Encoder", command=lambda:self._send("Q")).grid(row=0, column=8, padx=4, pady=6, sticky="ew")

        ttk.Label(ctrls, text="Steps:").grid(row=1, column=0, padx=(4,4), pady=6, sticky="e")
        ttk.Entry(ctrls, textvariable=self.steps_var, width=10).grid(row=1, column=1, padx=(0,8), sticky="w")
        ttk.Radiobutton(ctrls, text="Forward (+)", variable=self.dir_var, value="+").grid(row=1, column=2, padx=6, sticky="w")
        ttk.Radiobutton(ctrls, text="Reverse (-)", variable=self.dir_var, value="-").grid(row=1, column=3, padx=6, sticky="w")
        ttk.Button(ctrls, text="Move N (G)", command=self._move_n).grid(row=1, column=4, padx=4, sticky="ew")

        ttk.Label(ctrls, text="Presets:").grid(row=2, column=0, padx=4, pady=6, sticky="e")
        ttk.Button(ctrls, text="¼ Rev", command=lambda:self._move_frac(0.25, True)).grid(row=2, column=1, padx=4, sticky="ew")
        ttk.Button(ctrls, text="½ Rev", command=lambda:self._move_frac(0.50, True)).grid(row=2, column=2, padx=4, sticky="ew")
        ttk.Button(ctrls, text="1 Rev", command=lambda:self._move_frac(1.00, True)).grid(row=2, column=3, padx=4, sticky="ew")
        ttk.Button(ctrls, text="2 Rev", command=lambda:self._move_frac(2.00, True)).grid(row=2, column=4, padx=4, sticky="ew")
        ttk.Button(ctrls, text="5 Rev", command=lambda:self._move_frac(5.00, True)).grid(row=2, column=5, padx=4, sticky="ew")

        ttk.Label(ctrls, text="Jog:").grid(row=3, column=0, padx=4, pady=6, sticky="e")
        for i, val in enumerate([10, 100, 1000]):
            ttk.Button(ctrls, text=f"-{val}", command=lambda v=-val: self._send(f"G {v}")).grid(row=3, column=1+i, padx=4, sticky="ew")
        for i, val in enumerate([10, 100, 1000]):
            ttk.Button(ctrls, text=f"+{val}", command=lambda v=val: self._send(f"G {v}")).grid(row=3, column=4+i, padx=4, sticky="ew")

        # Log (always visible)
        logf = ttk.LabelFrame(self, text="Log")
        logf.grid(row=4, column=0, sticky="nsew")
        logf.rowconfigure(0, weight=1)
        logf.columnconfigure(0, weight=1)
        self.log = tk.Text(logf, height=10, wrap="word")
        self.log.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(logf)
        yscroll.config(command=self.log.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.log["yscrollcommand"] = yscroll.set


    def _home_motor(self):
        # Send homing command to Arduino
        self._send("H")
        self.motor_position = 0
        self.expected_position_var.set(str(self.motor_position))

    def _spin_motor(self):
        # Spin motor continuously in the selected direction at current speed
        try:
            steps_rev = int(float(self.steps_rev_var.get().strip()))
        except Exception:
            steps_rev = 400
        sign = 1 if self.dir_var.get() == "+" else -1
        # Large value for continuous spin
        if self.ser.is_open():
            self.ser.write_line(f"G {sign * 1000000}")

    def _stop_motor(self):
        # Stop motor immediately
        if self.ser.is_open():
            self.ser.write_line("Z")

    # Helpers
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            if self.port_var.get() in ports:
                pass  # keep current selection
            else:
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def _try_auto_connect_arduino(self):
        # Try to auto-connect to first port named 'Arduino Uno'
        ports = list(serial.tools.list_ports.comports())
        arduino_port = None
        for p in ports:
            if "Arduino Uno" in p.description:
                arduino_port = p.device
                break
        if arduino_port:
            self.port_var.set(arduino_port)
            self._connect()

    def _auto_refresh_ports(self):
        self._refresh_ports()
        self.after(3000, self._auto_refresh_ports)

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Select a COM port first.")
            return
        ok, err = self.ser.open(port, BAUD)
        if not ok:
            messagebox.showerror("Connect failed", f"{err}")
            return
        self.status_var.set(f"Connected: {port}")
        self._log(f"Connected to {port} @ {BAUD}")

    def _disconnect(self):
        self.ser.close()
        self.status_var.set("Disconnected")
        self._log("Disconnected.")

    def _send(self, line: str):
        try:
            self.ser.write_line(line)
            self._log(f"> {line.strip()}")
            # Update expected position for step commands
            cmd = line.strip().split()
            if cmd:
                steps_per_rev = int(self.steps_rev_var.get())
                if cmd[0] == "G" and len(cmd) > 1:
                    try:
                        steps = int(cmd[1])
                        self.motor_position = (self.motor_position + steps) % steps_per_rev
                        self.expected_position_var.set(str(self.motor_position))
                    except Exception:
                        pass
                elif cmd[0] == "R":
                    steps = steps_per_rev
                    self.motor_position = (self.motor_position + steps) % steps_per_rev
                    self.expected_position_var.set(str(self.motor_position))
                elif cmd[0] == "L":
                    steps = steps_per_rev
                    self.motor_position = (self.motor_position - steps) % steps_per_rev
                    self.expected_position_var.set(str(self.motor_position))
        except Exception as e:
            messagebox.showerror("Serial error", str(e))

    def _set_speed(self):
        mode = self.speed_mode_var.get()
        try:
            val = float(self.speed_val_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid", "Enter a numeric speed value.")
            return
        if mode == "RPM":
            steps_rev = max(1, int(float(self.steps_rev_var.get() or 200)))
            sps = val * steps_rev / 60.0
        else:
            sps = val
        sps = max(10.0, min(sps, 20000.0))
        self._send(f"V {sps:.3f}")

        

    def _set_accel(self):
        try:
            a = float(self.accel_val_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid", "Enter a numeric acceleration (steps/s²).")
            return
        a = clamp(a, 10.0, 50000.0)
        self._send(f"A {a:.3f}")


    def _move_n(self):
        try:
            n = int(self.steps_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid", "Enter an integer number of steps.")
            return
        sign = 1 if self.dir_var.get() == "+" else -1
        self._send(f"G {sign*n}")

    def _move_frac(self, revs: float, forward: bool):
        try:
            steps_rev = int(float(self.steps_rev_var.get().strip()))
            steps_rev = max(1, steps_rev)
        except ValueError:
            messagebox.showwarning("Invalid", "Steps/Rev must be a number.")
            return
        n = int(round(steps_rev * revs))
        sign = 1 if self.dir_var.get() == "+" else -1
        
        self._send(f"G {sign*n}")

    # Poll the serial port for incoming messages
    def _poll_serial(self):
        while True:
            try:
                line = self.ser.read_q.get_nowait()
            except queue.Empty:
                break
            line = line.strip()

            # Support SENSOR: 0/1 and SENSOR (digital): 0/1 messages from Arduino
            if line.startswith("SENSOR:"):
                value = line.split(":", 1)[1].strip()
                if value == "1":
                    self.pin5_state_var.set(True)
                elif value == "0":
                    self.pin5_state_var.set(False)
            if line.startswith("SENSOR (digital):"):
                value = line.split(":", 1)[1].strip()
                if value == "1":
                    self.pin5_state_var.set(True)
                elif value == "0":
                    self.pin5_state_var.set(False)

            #  POSITION: messages from Arduino encoder
            if line.startswith("POSITION:"):
                value = line.split(":", 1)[1].strip()
                try:
                    pos = int(value)
                    # Display true encoder position, reset to 0 after each full revolution (assuming 5.12 steps per degree)
                    self.encoder_position_var.set(str(pos))
                    # Calculate converted encoder position (e.g., pos / steps_per_rev)
                    self.converted_encoder_position_var.set(pos / 5.12)
                except Exception:
                    self.converted_encoder_position_var.set("ERR")

            self._log(line)
        self.after(50, self._poll_serial)

    def _log(self, s: str):
        if self.log:
            self.log.insert("end", s + "\n")
            self.log.see("end")
        else:
            print(s)

    def _on_close(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self.master.destroy()

def main():
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.0)
        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        elif "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:
        pass

    app = StepperApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()