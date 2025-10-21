import xml.etree.ElementTree as ET
import os
import time
import threading
import customtkinter as ctk
from tkinter import filedialog, messagebox
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# --- KONFIGURASI INFLUXDB CLOUD ---
INFLUXDB_URL = "https://us-east-1-1.aws.cloud2.influxdata.com"
INFLUXDB_TOKEN = "4iveQHlkSl7m-EArzJZKxVLXB5G9FP21VTcbujdk_pfm7NHBGel2DZDECdLt-LLNtTMvQXFI1qlx2clomBSeOw=="
INFLUXDB_ORG = "skt"
INFLUXDB_BUCKET = "skt1"

running = False  # Flag untuk loop pengiriman

# --- FUNGSI LOGIKA ---
def extract_temperatures_from_xml(file_path):
    """
    Membaca file XML DWSIM dan mengembalikan dictionary stream_name: temperature(°C)
    """
    temperatures = {}
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        # Pemetaan ID stream ke tag
        stream_tags = {}
        for graphic_object in root.findall(".//GraphicObject[ObjectType='MaterialStream']"):
            stream_id = graphic_object.find('Name').text
            stream_tag = graphic_object.find('Tag').text
            if stream_id and stream_tag:
                stream_tags[stream_id] = stream_tag

        # Ambil temperatur
        for sim_object in root.findall(".//SimulationObject[Type='DWSIM.Thermodynamics.Streams.MaterialStream']"):
            stream_id = sim_object.find('Name').text
            if stream_id in stream_tags:
                temp_kelvin_element = sim_object.find(".//Phase[ComponentName='Mixture']/Properties/temperature")
                if temp_kelvin_element is not None:
                    temp_kelvin = float(temp_kelvin_element.text)
                    temp_celsius = temp_kelvin - 273.15
                    stream_name = stream_tags[stream_id]
                    temperatures[stream_name] = temp_celsius

        return temperatures

    except Exception as e:
        messagebox.showerror("Error", f"Gagal membaca XML: {e}")
        return None

def send_to_influxdb(client, data):
    """
    Mengirim data temperatur ke InfluxDB Cloud
    """
    try:
        write_api = client.write_api(write_options=SYNCHRONOUS)
        points = []
        for stream_name, temp_celsius in data.items():
            point = (
                Point("heat_exchanger_monitoring")
                .tag("stream_name", stream_name)
                .field("temperature_celsius", float(f"{temp_celsius:.2f}"))
            )
            points.append(point)

        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=points)
        return True
    except Exception as e:
        messagebox.showerror("Error", f"Gagal mengirim ke InfluxDB Cloud: {e}")
        return False


# --- FUNGSI UI ---
def browse_file():
    file_path = filedialog.askopenfilename(filetypes=[("XML Files", "*.xml")])
    if file_path:
        xml_path_entry.delete(0, ctk.END)
        xml_path_entry.insert(0, file_path)

def start_monitoring():
    global running
    running = True
    start_button.configure(state="disabled")
    stop_button.configure(state="normal")
    thread = threading.Thread(target=monitor_loop)
    thread.daemon = True
    thread.start()

def stop_monitoring():
    global running
    running = False
    start_button.configure(state="normal")
    stop_button.configure(state="disabled")

def monitor_loop():
    global running

    file_path = xml_path_entry.get().strip()
    interval = int(interval_entry.get())

    if not os.path.exists(file_path):
        messagebox.showerror("Error", "File XML tidak ditemukan.")
        return

    client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)

    while running:
        data = extract_temperatures_from_xml(file_path)
        if data:
            update_display(data)
            success = send_to_influxdb(client, data)
            if success:
                status_label.configure(text=f"Data dikirim ke InfluxDB Cloud @ {time.strftime('%H:%M:%S')}", text_color="green")
        else:
            status_label.configure(text="Gagal membaca data dari XML.", text_color="red")

        for i in range(interval):
            if not running:
                break
            time.sleep(1)

def update_display(data):
    text_output.delete("1.0", ctk.END)
    for name, temp in data.items():
        text_output.insert(ctk.END, f"{name}: {temp:.2f} °C\n")


# --- SETUP UI ---
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

app = ctk.CTk()
app.title("DWSIM XML → InfluxDB Cloud Monitor")
app.geometry("700x500")

# Frame utama
frame = ctk.CTkFrame(app, corner_radius=10)
frame.pack(padx=20, pady=20, fill="both", expand=True)

# Path XML
ctk.CTkLabel(frame, text="Path File XML DWSIM:").pack(anchor="w")
xml_path_entry = ctk.CTkEntry(frame, width=500)
xml_path_entry.pack(side="left", padx=5, pady=5)
browse_btn = ctk.CTkButton(frame, text="Browse", command=browse_file)
browse_btn.pack(side="left", padx=5, pady=5)

# Interval
interval_frame = ctk.CTkFrame(app)
interval_frame.pack(pady=10)
ctk.CTkLabel(interval_frame, text="Interval Kirim (detik):").pack(side="left")
interval_entry = ctk.CTkEntry(interval_frame, width=60)
interval_entry.insert(0, "60")
interval_entry.pack(side="left", padx=5)

# Tombol
btn_frame = ctk.CTkFrame(app)
btn_frame.pack(pady=10)
start_button = ctk.CTkButton(btn_frame, text="Start", fg_color="green", command=start_monitoring)
start_button.pack(side="left", padx=10)
stop_button = ctk.CTkButton(btn_frame, text="Stop", fg_color="red", command=stop_monitoring, state="disabled")
stop_button.pack(side="left", padx=10)

# Output data
ctk.CTkLabel(app, text="Data Temperatur:").pack(anchor="w", padx=20)
text_output = ctk.CTkTextbox(app, height=200)
text_output.pack(padx=20, pady=10, fill="both", expand=True)

# Status
status_label = ctk.CTkLabel(app, text="Menunggu aksi...", text_color="yellow")
status_label.pack(pady=5)

app.mainloop()
