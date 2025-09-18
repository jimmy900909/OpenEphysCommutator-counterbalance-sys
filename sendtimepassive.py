import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import threading
import time
import csv
import serial.tools.list_ports
from datetime import datetime  # ✅ 新增

class HX711Logger:
    def __init__(self, root):
        self.root = root
        self.root.title("HX711 Loadcell Logger")
        self.root.geometry("400x250")

        # 狀態變數
        self.ser = None
        self.is_logging = False
        self.thread = None
        self.output_file = "tension_log.csv"

        # --- UI 元件 ---
        # COM Port 選擇
        tk.Label(root, text="COM Port:").pack(pady=5)
        self.combobox_port = ttk.Combobox(root, values=self.get_serial_ports(), state="readonly")
        self.combobox_port.pack()

        # Baudrate
        tk.Label(root, text="Baudrate:").pack(pady=5)
        self.baud_entry = tk.Entry(root)
        self.baud_entry.insert(0, "9600")
        self.baud_entry.pack()

        # 當前數據
        self.label_data = tk.Label(root, text="Current Data: ---", font=("Arial", 14))
        self.label_data.pack(pady=10)

        # 按鈕
        self.start_btn = tk.Button(root, text="Start Logging", command=self.start_logging, bg="green", fg="white")
        self.start_btn.pack(pady=5)

        self.stop_btn = tk.Button(root, text="Stop Logging", command=self.stop_logging, bg="red", fg="white", state=tk.DISABLED)
        self.stop_btn.pack(pady=5)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def start_logging(self):
        port = self.combobox_port.get()
        baud = self.baud_entry.get()

        if not port:
            messagebox.showerror("Error", "請選擇 COM Port")
            return

        try:
            baud = int(baud)
        except ValueError:
            messagebox.showerror("Error", "Baudrate 必須是數字")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            messagebox.showerror("Error", f"無法開啟序列埠: {e}")
            return

        # 選擇輸出檔案
        self.output_file = filedialog.asksaveasfilename(defaultextension=".csv",
                                                        filetypes=[("CSV files", "*.csv")],
                                                        title="選擇輸出檔案")
        if not self.output_file:
            self.ser.close()
            return

        self.is_logging = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        self.thread = threading.Thread(target=self.log_data)
        self.thread.daemon = True
        self.thread.start()

    def log_data(self):
        with open(self.output_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "RawReading", "Weight_g"])  # 標頭

            raw = "---"  # 避免還沒讀到 Raw 時就寫入

            while self.is_logging:
                try:
                    line = self.ser.readline().decode("utf-8").strip()
                    if line:
                        if line.startswith("Raw reading:"):
                            raw = line.split(":")[1].strip()
                        elif line.startswith("Weight:"):
                            weight = line.split(":")[1].strip().split()[0]
                            # ✅ 毫秒時間戳記
                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            writer.writerow([timestamp, raw, weight])
                            file.flush()  # ✅ 即時寫入
                            self.label_data.config(text=f"Current Data: {weight} g")
                except Exception as e:
                    print("Error reading serial:", e)
                    break
        self.ser.close()

    def stop_logging(self):
        self.is_logging = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        messagebox.showinfo("Info", f"紀錄完成，已存檔：{self.output_file}")

if __name__ == "__main__":
    root = tk.Tk()
    app = HX711Logger(root)
    root.mainloop()

