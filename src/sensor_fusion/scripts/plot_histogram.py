import pandas as pd
import matplotlib.pyplot as plt
import rospy
import sys

class ParticleFilterPlotter:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = None
        plt.ion()  # Interaktiven Modus aktivieren

    def read_data(self):
        # Lesen der CSV-Datei in ein DataFrame und Konvertieren der Daten in numerische Typen
        self.data = pd.read_csv(self.file_path, names=["x", "y", "theta", "weight"])
        self.data = self.data.apply(pd.to_numeric, errors='coerce')

    def plot_histogram(self):
        if self.data is None:
            raise ValueError("Data not loaded. Call read_data() first.")

        # Erstellen Sie ein Histogramm für die Gewichtswerte der Partikel
        plt.clf()  # Löschen der aktuellen Figur
        plt.hist(self.data["weight"].dropna(), bins=10, edgecolor='black')
        plt.xlabel('Weight Bins')
        plt.ylabel('Anzahl der Partikel')
        plt.title('Histogramm der Partikelgewichte')
        plt.xlim(0, 1)  # Setzen Sie die X-Achse auf den Bereich von 0 bis 1
        plt.draw()
        plt.pause(0.001)  # Kurze Pause, um den Plot zu aktualisieren

if __name__ == '__main__':
    rospy.init_node('particle_histogram_plotter', anonymous=True)

    # Lesen des Dateipfads von den ROS-Parametern
    file_path = rospy.get_param('~file_path', 'src/sensor_fusion/measurements/Particle_Resampling_Histogramm.csv')
    
    plotter = ParticleFilterPlotter(file_path)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        plotter.read_data()
        plotter.plot_histogram()
        rate.sleep()

    plt.ioff()  # Interaktiven Modus deaktivieren
    plt.show()  # Zeigen Sie den letzten Plot an, wenn das Programm endet
