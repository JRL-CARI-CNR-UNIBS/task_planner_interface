#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import String, Bool, Float32


class TaskRequestApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Task Request App")

        self.root.geometry("400x300")  # Imposta la dimensione dell'interfaccia grafica

        self.request_label = tk.Label(root, text="Task Request:")
        self.request_label.grid(row=0, column=0, sticky="w", padx=10, pady=(10, 0))

        self.request_text = tk.StringVar()
        self.request_entry = tk.Entry(root, textvariable=self.request_text)
        self.request_entry.grid(row=1, column=0, columnspan=2, padx=10, pady=(0, 10), sticky="ew")

        # Aggiungi uno spazio vuoto sopra al pulsante
        self.empty_label = tk.Label(root, text="")
        self.empty_label.grid(row=2, column=0)

        self.send_button = tk.Button(root, text="Send Feedback", command=self.send_feedback, bg="#3498db", fg="white",
                                     padx=5, pady=5)
        self.send_button.grid(row=3, column=0, columnspan=2, pady=(0, 10))

        # Configura il layout della griglia in modo che le celle si espandano con la finestra
        root.grid_rowconfigure(3, weight=1)
        root.grid_columnconfigure(0, weight=1)

        # Inizializza il nodo ROS
        rospy.init_node('task_request_app', anonymous=True)

        # Crea un subscriber per ricevere la richiesta
        rospy.Subscriber('gui/request', String, self.request_callback)

        # Crea un publisher per inviare la conferma di esecuzione
        self.execution_publisher = rospy.Publisher('gui/feedback', Bool, queue_size=10)

        # Crea un subscriber per leggere il valore dell'avanzamento
        rospy.Subscriber('completition_info_human', Float32, self.progress_callback)

        # Variabile per il valore dell'avanzamento
        self.progress_value = tk.DoubleVar()
        self.progressbar = ttk.Progressbar(root, mode='determinate', variable=self.progress_value, maximum=1.0)
        self.progressbar.grid(row=4, column=0, columnspan=2, pady=10, padx=10, sticky="ew")

        # Etichetta per mostrare il valore in percentuale
        self.percent_label = tk.Label(root, text="")
        self.percent_label.grid(row=5, column=0, columnspan=2)

        # Gestione dell'evento di chiusura della finestra
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def request_callback(self, data):
        # Callback chiamato quando arriva una nuova richiesta
        # Aggiorna l'interfaccia grafica con la richiesta ricevuta
        self.request_text.set(f"TODO: {data.data}")
        if "end" in data.data:
            rospy.sleep(1)
            self.execution_publisher.publish(True)
            self.request_text.set("")

    def send_feedback(self):
        # Invia la conferma di esecuzione sul topic ROS
        self.request_text.set("")
        self.progress_value.set(0)
        self.percent_label.config(text="")

        self.execution_publisher.publish(True)

    def progress_callback(self, data):
        # Callback chiamato quando arriva un nuovo valore di avanzamento
        # Aggiorna il valore della barra di avanzamento
        progress_value = data.data
        self.progress_value.set(progress_value / 100)

        # Calcola il valore in percentuale
        percent_value = progress_value

        # Aggiorna il testo della percent_label
        self.percent_label.config(text=f"{percent_value:.2f}%")

    def on_closing(self):
        # Funzione chiamata quando si preme la "x" della finestra
        rospy.signal_shutdown("Chiusura tramite interfaccia grafica")
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = TaskRequestApp(root)
    root.mainloop()
