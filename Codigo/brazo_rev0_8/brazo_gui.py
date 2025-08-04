import tkinter as tk
from tkinter import filedialog, messagebox
import serial
import threading
import time

class BrazoCopiadorApp:
    def __init__(self, master):
        self.master = master
        master.title("Brazo Copiador - Grabación y Reproducción")

        self.serial_port = tk.StringVar(value="COM6")
        self.baudrate = 115200
        self.ser = None

        # Config del sistema mecánico
        self.PASOS_REV = 1600              # 1/8 microstepping
        self.STEP_ANGLE = 360.0 / self.PASOS_REV  # 0.225°/paso

        # Variable UI
        self.current_pos = tk.StringVar(value="---")
        self.quantize_var = tk.BooleanVar(value=False)  # cuantizar a resolución de paso

        self.build_gui()

    def build_gui(self):
        frame = tk.Frame(self.master)
        frame.pack(padx=20, pady=20)

        # Conexión
        tk.Label(frame, text="Puerto Serial:").grid(row=0, column=0)
        tk.Entry(frame, textvariable=self.serial_port, width=10).grid(row=0, column=1)
        tk.Button(frame, text="Conectar", command=self.conectar).grid(row=0, column=2)

        # Acciones principales
        tk.Button(frame, text="Grabar trayectoria", command=self.grabar_trayectoria).grid(row=1, column=0, columnspan=3, pady=2)
        tk.Button(frame, text="Reproducir trayectoria", command=self.reproducir_trayectoria).grid(row=2, column=0, columnspan=3, pady=2)
        tk.Button(frame, text="Hacer Homing", command=self.enviar_homing).grid(row=3, column=0, columnspan=3, pady=2)
        tk.Button(frame, text="Enable", command=self.toggle_enable).grid(row=4, column=0, pady=2)
        tk.Button(frame, text="Disable", command=self.toggle_disable).grid(row=4, column=1, pady=2)
        tk.Button(frame, text="Detener Grabación", command=self.detener_grabacion).grid(row=5, column=0, columnspan=3, pady=2)

        # Opciones
        tk.Checkbutton(frame, text="Cuantizar a paso (0.225°)", variable=self.quantize_var).grid(row=6, column=0, columnspan=2, sticky="w")

        # Posición actual
        tk.Label(frame, text="Posición Actual (°):").grid(row=7, column=0)
        tk.Label(frame, textvariable=self.current_pos, width=14, relief="sunken").grid(row=7, column=1)
        tk.Button(frame, text="Actualizar posición", command=self.mostrar_posicion).grid(row=7, column=2, pady=2)

    # ------------------------ Helpers serial ---------------------
    def conectar(self):
        try:
            self.ser = serial.Serial(self.serial_port.get(), self.baudrate, timeout=1)
            time.sleep(2)
            messagebox.showinfo("Conectado", f"Conectado a {self.serial_port.get()}")
        except Exception as e:
            messagebox.showerror("Error de conexión", str(e))

    def _wait_for(self, token: str, timeout=5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if token in line:
                return True
        return False

    # ------------------------ Grabación --------------------------
    def grabar_trayectoria(self):
        if not self.ser:
            messagebox.showerror("Error", "Primero conectá el puerto serial.")
            return
        archivo = filedialog.asksaveasfilename(defaultextension=".csv",
                                               filetypes=[("CSV files", "*.csv")])
        if not archivo:
            return

        def grabar():
            # Preparar puerto
            self.ser.timeout = 0.8      # lectura ágil
            self.ser.reset_input_buffer()
            self.ser.write(b"g")        # pedir grabación en Arduino

            grabando = False
            last_rx = time.time()       # último paquete recibido (se resetea con cada línea)

            try:
                with open(archivo, "w") as f:
                    while True:
                        raw = self.ser.readline()  # bloquea hasta timeout (0.8 s)
                        if raw:
                            last_rx = time.time()  # llegó algo -> reseteo watchdog
                            linea = raw.decode(errors="ignore").strip()
                            if linea == "INICIO_TRAYECTORIA":
                                grabando = True
                                continue
                            if linea == "FIN_TRAYECTORIA":
                                break
                            if grabando and "," in linea:
                                try:
                                    ang1, ang2 = linea.split(",", 1)
                                    float(ang1); float(ang2)  # valida
                                    f.write(f"{ang1},{ang2}\n")
                                except ValueError:
                                    pass
                        else:
                            # No llegó nada en 'timeout'; verifico inactividad prolongada
                            if time.time() - last_rx > 3.0:  # 3 s sin datos
                                # mejor enviar 'x' para terminar polite
                                try: self.ser.write(b"x")
                                except: pass
                                # Mostramos el error en el hilo principal de Tk
                                self.master.after(0, lambda: messagebox.showerror(
                                    "Timeout",
                                    "No llegan datos del Arduino hace 3 s. ¿Está enviando 'INICIO_TRAYECTORIA' y las muestras?"))
                                return
            finally:
                # Mensaje final seguro en el hilo principal
                self.master.after(0, lambda: messagebox.showinfo("Listo", f"Trayectoria guardada en {archivo}"))

        threading.Thread(target=grabar, daemon=True).start()


    # ------------------------ Reproducción -----------------------
    def reproducir_trayectoria(self):
        if not self.ser:
            messagebox.showerror("Error", "Primero conectá el puerto serial.")
            return
        archivo = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not archivo:
            return
        def enviar():
            self.ser.reset_input_buffer()
            self.ser.write(b"r")  # invoca recibirYReproducirSerial()
            if not self._wait_for("RDY", timeout=2.0):
                messagebox.showerror("Error", "Arduino no respondió RDY.")
                return

            self.ser.write(b"INICIO_TRAYECTORIA\n")
            with open(archivo, "r") as f:
                for linea in f:
                    punto = linea.strip()
                    if not punto:
                        continue
                    if self.quantize_var.get():
                        try:
                            a1s, a2s = punto.split(",", 1)
                            a1 = float(a1s); a2 = float(a2s)
                            a1q = round(a1 / self.STEP_ANGLE) * self.STEP_ANGLE
                            a2q = round(a2 / self.STEP_ANGLE) * self.STEP_ANGLE
                            punto = f"{a1q:.3f},{a2q:.3f}"
                        except Exception:
                            pass
                    self.ser.write((punto + "\n").encode())
                    if not self._wait_for("OK", timeout=10.0):
                        messagebox.showerror("Error", f"Sin ACK para punto: {punto}")
                        return
            self.ser.write(b"FIN_TRAYECTORIA\n")
            self._wait_for("DONE", timeout=3.0)
            messagebox.showinfo("Listo", "Trayectoria enviada y ejecutada.")
        threading.Thread(target=enviar, daemon=True).start()

    # ------------------------ Comandos varios --------------------
    def enviar_homing(self):
        if self.ser:
            self.ser.write(b"h")

    def toggle_enable(self):
        if self.ser:
            self.ser.write(b"e")

    def toggle_disable(self):
        if self.ser:
            self.ser.write(b"d")

    def mostrar_posicion(self):
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.write(b"p")
            time.sleep(0.05)
            datos = self.ser.readline().decode().strip()
            self.current_pos.set(datos if datos else "---")

    def detener_grabacion(self):
        if self.ser:
            self.ser.write(b"x")

if __name__ == "__main__":
    root = tk.Tk()
    app = BrazoCopiadorApp(root)
    root.mainloop()
