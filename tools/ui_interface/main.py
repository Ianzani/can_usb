import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import tkinter as tk
from threading import Thread
import time
import struct

SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200
LEN_RING_BUFFER = 3000

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ---------------- Uart Buffer ---------------- #
uart_buffer = []

# ---------------- Value buffers ---------------- #
speed_ref_buffer = []
speed_read_buffer = []
current_read_buffer = []
dc_voltage_read_buffer = []

run_flag = True

def read_serial():
    global run_flag
    while run_flag:
        try:
            bytes_read = ser.read(16)
            for i in range(len(bytes_read)):
                byte_read = bytes_read[i:i+1]  # Get 1 byte
                value = struct.unpack('B', byte_read)[0]
                uart_buffer.append(value)
                if len(uart_buffer) > 10:
                    uart_buffer.pop(0)

                if len(uart_buffer) >= 10:
                    if uart_buffer[0] == 0x4e and uart_buffer[9] == 0x9a:
                        print((uart_buffer[1] | (uart_buffer[2] << 8)) / 100)
                        speed_ref_buffer.append((uart_buffer[1] | (uart_buffer[2] << 8)) / 100)
                        speed_read_buffer.append((uart_buffer[3] | (uart_buffer[4] << 8)) / 100)
                        current_read_buffer.append((uart_buffer[5] | (uart_buffer[6] << 8)) / 100)
                        dc_voltage_read_buffer.append((uart_buffer[7] | (uart_buffer[8] << 8)) / 100)

                        if len(speed_ref_buffer) > LEN_RING_BUFFER:
                            speed_ref_buffer.pop(0)

                        if len(speed_read_buffer) > LEN_RING_BUFFER:
                            speed_read_buffer.pop(0)

                        if len(current_read_buffer) > LEN_RING_BUFFER:
                            current_read_buffer.pop(0)

                        if len(dc_voltage_read_buffer) > LEN_RING_BUFFER:
                            dc_voltage_read_buffer.pop(0)

        except Exception as e:
            print(f'UART READ ERROR: {e}')

        time.sleep(0.001) # 1ms


def send_serial(value):
    try:
        bytes_to_send = struct.pack('<H', value)  # '<H' little endian 
        
        bytes_to_send = bytes([0x4e]) + bytes_to_send + bytes([0x9a])
        ser.write(bytes_to_send)
    except Exception as e:
        print(f'UART SEND ERROR: {e}')


def update(frame):
    # Speed plot
    axs[0].clear()
    axs[0].plot(speed_ref_buffer, label='Referência', color='b', linestyle='--', linewidth=2)
    axs[0].plot(speed_read_buffer, label='Atual', color='r', linewidth=2)
    axs[0].set_title('Velocidade do Motor', fontsize=14)
    axs[0].legend()
    axs[0].set_ylabel('Velocidade (RPM)', fontsize=12)
    axs[0].grid(True)

    # Current plot
    axs[1].clear()
    axs[1].plot(current_read_buffer, color='g', linewidth=2)
    axs[1].set_title('Corrente do Estator', fontsize=14)
    axs[1].set_ylabel('Corrente (A)', fontsize=12)
    axs[1].grid(True)

    #Voltade plot
    axs[2].clear()
    axs[2].plot(dc_voltage_read_buffer, color='m', linewidth=2)
    axs[2].set_title('Tensão de Barramento', fontsize=14)
    axs[2].set_xlabel('Tempo', fontsize=12)
    axs[2].set_ylabel('Tensão (V)', fontsize=12)
    axs[2].grid(True)

def on_send():
    try:
        user_input = int(entry.get())
        if 0 <= user_input <= 1715: # Nominal speed
            send_serial(round(user_input * 2 * np.pi * 100 / 60)) # RMP to rad/s
            entry.delete(0, tk.END)
        else:
            print("Por favor, insira um número entre 0 e 1710.")
    except ValueError:
        print("Por favor, insira um número válido.")


def on_closing():
    global run_flag
    run_flag = False
    ser.close()
    root.destroy()

# UI configuration
root = tk.Tk()
root.title("Interface Serial")
root.protocol("WM_DELETE_WINDOW", on_closing)

# Plot configuration
fig, axs = plt.subplots(3, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [1, 0.5, 0.5]})  # Cria um gráfico de 3 linhas
plt.subplots_adjust(hspace=0.4)  # Aumenta o espaço vertical entre os gráficos

# Input box
entry_label = tk.Label(root, text="Velocidade de referência (RPM):")
entry_label.grid(row=1, column=0, padx=10, pady=5)

entry = tk.Entry(root)
entry.grid(row=1, column=1, padx=10, pady=5)

# Send button
send_button = tk.Button(root, text="Enviar Valor", command=on_send)
send_button.grid(row=2, column=0, columnspan=2, pady=10)

# Plot update
ani = FuncAnimation(fig, update, interval=100)  # Update each 100ms

# Start serial thread
Thread(target=read_serial, daemon=True).start()

# Start UI
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=0, column=0, columnspan=2, sticky='nsew')
root.mainloop()
