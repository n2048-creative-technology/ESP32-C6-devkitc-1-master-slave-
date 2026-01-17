#!/home/mauricio/Documents/leaf-remote-control/.venv/bin/python3

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover - handled at runtime
    serial = None


DEFAULT_BAUD = 115200
SLAVES_RE = re.compile(r"slaves=(\d+)")


class SerialWorker(threading.Thread):
    def __init__(self, port, baud, out_q, stop_evt, write_q):
        super().__init__(daemon=True)
        self._port = port
        self._baud = baud
        self._out_q = out_q
        self._stop_evt = stop_evt
        self._write_q = write_q
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud, timeout=0.2, write_timeout=1.0
            )
            self._ser.reset_output_buffer()
            self._out_q.put(("status", f"Connected to {self._port} @ {self._baud}"))
        except Exception as exc:
            self._out_q.put(("error", f"Failed to open {self._port}: {exc}"))
            return

        buf = b""
        while not self._stop_evt.is_set():
            try:
                out = self._write_q.get_nowait()
            except queue.Empty:
                out = None
            if out:
                try:
                    self._ser.write(out)
                except Exception as exc:
                    self._out_q.put(("error", f"Serial write error: {exc}"))
                    break

            try:
                chunk = self._ser.read(256)
            except Exception as exc:
                self._out_q.put(("error", f"Serial read error: {exc}"))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    text = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    text = str(line)
                if text:
                    self._out_q.put(("line", text))

        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._out_q.put(("status", "Disconnected"))

    def write_line(self, text):
        if self._ser is None:
            return
        data = (text.strip() + "\n").encode("utf-8")
        try:
            self._write_q.put_nowait(data)
        except queue.Full:
            self._out_q.put(("error", "Write queue full"))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Leaf Master Monitor")
        self.geometry("640x520")

        self._worker = None
        self._stop_evt = threading.Event()
        self._queue = queue.Queue()
        self._write_queue = queue.Queue(maxsize=4)

        self._slaves_var = tk.StringVar(value="0")
        self._status_var = tk.StringVar(value="Disconnected")

        self._speed_var = tk.StringVar(value="0")
        self._prob_var = tk.StringVar(value="0")
        self._mode_var = tk.StringVar(value="0")

        self._port_var = tk.StringVar(value="/dev/ttyACM0")
        self._baud_var = tk.StringVar(value=str(DEFAULT_BAUD))

        self._build_ui()
        self._poll_queue()

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        conn_frame = ttk.LabelFrame(self, text="Connection")
        conn_frame.pack(fill="x", **pad)

        ttk.Label(conn_frame, text="Port").grid(row=0, column=0, sticky="w", **pad)
        port_entry = ttk.Entry(conn_frame, textvariable=self._port_var, width=20)
        port_entry.grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(conn_frame, text="Baud").grid(row=0, column=2, sticky="w", **pad)
        baud_entry = ttk.Entry(conn_frame, textvariable=self._baud_var, width=10)
        baud_entry.grid(row=0, column=3, sticky="w", **pad)

        refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports)
        refresh_btn.grid(row=0, column=4, sticky="w", **pad)

        self._connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self._connect_btn.grid(row=0, column=5, sticky="e", **pad)

        status_frame = ttk.LabelFrame(self, text="Status")
        status_frame.pack(fill="x", **pad)

        ttk.Label(status_frame, text="Slaves").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(status_frame, textvariable=self._slaves_var).grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(status_frame, text="Link").grid(row=0, column=2, sticky="w", **pad)
        ttk.Label(status_frame, textvariable=self._status_var).grid(row=0, column=3, sticky="w", **pad)

        params_frame = ttk.LabelFrame(self, text="Parameters")
        params_frame.pack(fill="x", **pad)

        ttk.Label(params_frame, text="Speed (0..1)").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(params_frame, textvariable=self._speed_var, width=10).grid(
            row=0, column=1, sticky="w", **pad
        )

        ttk.Label(params_frame, text="Probability (0..1)").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(params_frame, textvariable=self._prob_var, width=10).grid(
            row=0, column=3, sticky="w", **pad
        )

        ttk.Label(params_frame, text="Mode (0/1)").grid(row=0, column=4, sticky="w", **pad)
        ttk.Entry(params_frame, textvariable=self._mode_var, width=6).grid(
            row=0, column=5, sticky="w", **pad
        )

        self._send_btn = ttk.Button(params_frame, text="Send", command=self._send_params)
        self._send_btn.grid(row=0, column=6, sticky="e", **pad)
        self._send_btn.config(state="disabled")

        legend = ttk.Label(
            params_frame,
            text="Mode 0 = continuous (always runs), Mode 1 = probabilistic (uses probability)",
        )
        legend.grid(row=1, column=0, columnspan=7, sticky="w", **pad)

        log_frame = ttk.LabelFrame(self, text="Logs")
        log_frame.pack(fill="both", expand=True, **pad)

        self._log = tk.Text(log_frame, height=16, wrap="none", state="disabled")
        self._log.pack(fill="both", expand=True, padx=6, pady=6)

    def _refresh_ports(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is required.")
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._worker and self._worker.is_alive():
            self._stop_evt.set()
            return

        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial first.")
            return

        port = self._port_var.get().strip()
        if not port:
            messagebox.showerror("Port required", "Please set a serial port.")
            return

        try:
            baud = int(self._baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Baud invalid", "Baud must be an integer.")
            return

        self._stop_evt.clear()
        self._worker = SerialWorker(
            port, baud, self._queue, self._stop_evt, self._write_queue
        )
        self._worker.start()
        self._connect_btn.config(text="Disconnect")
        self._send_btn.config(state="normal")

    def _send_params(self):
        if not self._worker or not self._worker.is_alive():
            messagebox.showerror("Not connected", "Connect first.")
            return
        try:
            speed = float(self._speed_var.get().strip())
            prob = float(self._prob_var.get().strip())
            mode = int(self._mode_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid parameters", "Use numeric values.")
            return
        if speed < 0.0 or speed > 1.0 or prob < 0.0 or prob > 1.0 or mode not in (0, 1):
            messagebox.showerror("Out of range", "Speed/Probability 0..1, Mode 0/1.")
            return
        line = f"speed={speed:.3f},prob={prob:.3f},mode={mode}"
        try:
            while True:
                self._write_queue.get_nowait()
        except queue.Empty:
            pass
        self._worker.write_line(line)
        self._append_log(f"> {line}")

    def _poll_queue(self):
        while True:
            try:
                kind, payload = self._queue.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self._append_log(payload)
                match = SLAVES_RE.search(payload)
                if match:
                    self._slaves_var.set(match.group(1))
            elif kind == "status":
                self._status_var.set(payload)
                if payload == "Disconnected":
                    self._connect_btn.config(text="Connect")
                    self._send_btn.config(state="disabled")
            elif kind == "error":
                self._append_log(f"! {payload}")
                self._status_var.set("Error")
                self._connect_btn.config(text="Connect")
                self._send_btn.config(state="disabled")
        self.after(100, self._poll_queue)

    def _append_log(self, text):
        self._log.configure(state="normal")
        self._log.insert("end", text + "\n")
        self._log.see("end")
        self._log.configure(state="disabled")


if __name__ == "__main__":
    app = App()
    app.mainloop()
