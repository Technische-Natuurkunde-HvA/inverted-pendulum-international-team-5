import os
import time
import serial
import pandas as pd
import matplotlib.pyplot as plt

# ==========================
# PARAMÈTRES À ADAPTER
# ==========================
PORT = "COM5"        # port série de l'Arduino
BAUD = 115200        # baudrate identique à celui du code Arduino
DURATION = 10        # durée de l'acquisition en secondes

# ==========================
# CHEMIN DU CSV
# ==========================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(BASE_DIR, "data.csv")

print(f"Script lancé depuis : {BASE_DIR}")
print(f"Fichier CSV : {csv_path}")

# ==========================
# 1) ACQUISITION SUR COM8
# ==========================
print(f"Ouverture du port série {PORT} à {BAUD} bauds...")

with serial.Serial(PORT, BAUD, timeout=1) as ser, open(csv_path, "w") as f:
    # laisser le temps à l'Arduino de reset
    time.sleep(2.0)

    print(f"Acquisition pendant {DURATION} s...")
    t0 = time.time()

    while time.time() - t0 < DURATION:
        line_bytes = ser.readline()
        if not line_bytes:
            continue

        # décodage
        line = line_bytes.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        # contrôle du format : t,angle,rpm
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 3:
            print("Ligne ignorée (format inattendu) :", line)
            continue

        # écriture brute dans le CSV
        f.write(",".join(parts) + "\n")
        f.flush()

        print("OK :", parts)

print("Acquisition terminée.")
print(f"Données enregistrées dans : {csv_path}")

# ==========================
# 2) LECTURE AVEC PANDAS
# ==========================
print("Lecture du CSV avec pandas...")

df = pd.read_csv(csv_path, header=None, names=["t", "angle", "rpm"])
print(df.head())

# ==========================
# 3) PLOTS
# ==========================
time_s = df["t"]
angle_deg = df["angle"]
rpm = df["rpm"]

# --- Angle ---
plt.figure(figsize=(12, 5))
plt.plot(time_s, angle_deg, linewidth=2, label="Angle (deg)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# --- RPM ---
plt.figure(figsize=(12, 5))
plt.plot(time_s, rpm, linewidth=2, label="RPM")
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
