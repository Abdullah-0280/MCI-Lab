#!/usr/bin/env python3
"""
Simulation of LAB08_TASK2 and LAB08_TASK3 Embedded Systems Programs
This script simulates the gyroscope and temperature sensor readings
that would be output by the STM32 microcontroller programs.
"""

import time
import random
import math
from datetime import datetime
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class GyroscopeReading:
    """Container for gyroscope sensor data"""
    temperature: float  # Can be float for TASK2, int for TASK3
    x_rotation: float
    y_rotation: float
    z_rotation: float


class GyroscopeSensorSimulator:
    """
    Simulates L3GD20 Gyroscope sensor readings
    Provides realistic gyroscope and temperature data variations
    """
    
    def __init__(self, base_temperature: int = 28, simulation_mode: str = "default"):
        """
        Initialize sensor simulator
        
        Args:
            base_temperature: Base temperature in Celsius
            simulation_mode: "task2_stable" for TASK2 stable fluctuation, "task3_stepped" for TASK3 stepped pattern
        """
        self.base_temperature = base_temperature
        self.current_temp = base_temperature
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.time_step = 0
        self.simulation_mode = simulation_mode
        
    def generate_sensor_noise(self, base_value: float, noise_level: float = 2.0) -> float:
        """Generate realistic sensor noise using Gaussian distribution"""
        return base_value + random.gauss(0, noise_level)
    
    def update_gyroscope_data_task2_stable(self):
        """Update for TASK2: stable temperature with small fluctuations around 20-21°C"""
        # Small fluctuations around a stable value (20-21°C)
        stable_base = 20.5
        self.current_temp = max(19.5, min(22.5, stable_base + random.gauss(0, 0.5)))
        
        # Very low velocity readings, small noise
        self.current_x = random.gauss(0.1, 0.3)
        self.current_y = random.gauss(0.05, 0.25)
        self.current_z = random.gauss(-0.05, 0.3)
        
        self.time_step += 1
    
    def update_gyroscope_data_task3_stepped(self):
        """Update for TASK3: stepped temperature changes with small velocity fluctuations"""
        # Temperature steps based on time
        if self.time_step < 150:
            temp_base = 12.5
        elif self.time_step < 250:
            temp_base = 11.5
        else:
            temp_base = 10.0
        
        self.current_temp = temp_base + random.gauss(0, 0.15)
        
        # Low fluctuating velocities similar to graph
        time_normalized = self.time_step / 400.0
        self.current_x = math.sin(time_normalized * 6.28) * 0.8 + random.gauss(0, 0.4)
        self.current_y = math.cos(time_normalized * 4.5) * 0.6 + random.gauss(0, 0.35)
        self.current_z = math.sin(time_normalized * 3.2 + math.pi) * 0.7 + random.gauss(0, 0.4)
        
        self.time_step += 1
    
    def update_gyroscope_data(self):
        """Update gyroscope readings based on simulation mode"""
        if self.simulation_mode == "task2_stable":
            self.update_gyroscope_data_task2_stable()
        elif self.simulation_mode == "task3_stepped":
            self.update_gyroscope_data_task3_stepped()
        else:
            # Default mode - realistic patterns
            time_factor = math.sin(self.time_step * 0.05) * 15
            secondary_factor = math.cos(self.time_step * 0.03) * 10
            
            self.current_x = self.generate_sensor_noise(time_factor, 1.5)
            self.current_y = self.generate_sensor_noise(secondary_factor, 1.2)
            self.current_z = self.generate_sensor_noise(
                math.sin(self.time_step * 0.07) * 12, 1.8
            )
            
            self.current_temp = self.base_temperature + int(self.time_step * 0.02)
            self.time_step += 1
    
    def read_temperature_register(self):
        """Read temperature sensor value (simulating register 0x26)"""
        if self.simulation_mode == "task2_stable":
            # Return float value for TASK2
            return round(self.current_temp, 1)
        else:
            # Return int value for TASK3
            return int(self.current_temp)
    
    def read_axis_values(self) -> GyroscopeReading:
        """Read all axis values with temperature"""
        self.update_gyroscope_data()
        return GyroscopeReading(
            temperature=self.read_temperature_register(),
            x_rotation=self.current_x,
            y_rotation=self.current_y,
            z_rotation=self.current_z
        )


class TaskTwoSimulator:
    """
    Simulates LAB08_TASK2 behavior
    Reads temperature via interrupt-driven SPI communication
    Processes temperature data and outputs via UART
    """
    
    def __init__(self, gyro_sim: GyroscopeSensorSimulator):
        """
        Initialize TASK2 simulator
        
        Args:
            gyro_sim: Gyroscope simulator instance
        """
        self.gyro_sim = gyro_sim
        self.transfer_active = False
        self.sensor_data_byte = 0
        
    def convert_temperature_display(self, raw_value: int) -> int:
        """
        Convert raw sensor byte to display temperature
        Formula: displayValue = -(raw_value as signed) + 43
        Note: For simulation, we show raw fluctuating values
        """
        # Return raw temperature for display (before conversion formula)
        return round(raw_value, 1)
    
    def request_temperature_sample(self) -> str:
        """Simulate requesting and receiving temperature sample"""
        if self.transfer_active:
            return ""  # Skip if transfer already active
        
        self.transfer_active = True
        
        # Simulating transmit interrupt followed by receive interrupt
        reading = self.gyro_sim.read_axis_values()
        self.sensor_data_byte = reading.temperature
        
        # After receive completion callback - display raw temperature
        display_temp = self.sensor_data_byte if isinstance(self.sensor_data_byte, float) else float(self.sensor_data_byte)
        self.transfer_active = False
        
        return f"{display_temp}\r\n"
    
    def run_simulation(self, iterations: int = 5) -> list:
        """
        Run complete TASK2 simulation
        
        Args:
            iterations: Number of temperature samples to generate
            
        Returns:
            List of output strings
        """
        outputs = []
        for _ in range(iterations):
            output = self.request_temperature_sample()
            if output:
                outputs.append(output)
            time.sleep(0.001)  # Simulate 1-second delay between samples in real hardware
        
        return outputs


class TaskThreeSimulator:
    """
    Simulates LAB08_TASK3 behavior
    Reads temperature and XYZ rotation data via polling SPI
    Outputs formatted data string via UART
    """
    
    def __init__(self, gyro_sim: GyroscopeSensorSimulator):
        """
        Initialize TASK3 simulator
        
        Args:
            gyro_sim: Gyroscope simulator instance
        """
        self.gyro_sim = gyro_sim
        
    def acquire_all_sensor_data(self) -> str:
        """
        Acquire temperature and axis data, format for UART output
        
        Returns:
            Formatted output string: "TEMP,X_DPS,Y_DPS,Z_DPS"
        """
        reading = self.gyro_sim.read_axis_values()
        
        # Format: temperature,x_rotation,y_rotation,z_rotation
        output = f"{reading.temperature},{reading.x_rotation:.2f},"
        output += f"{reading.y_rotation:.2f},{reading.z_rotation:.2f}\r\n"
        
        return output
    
    def run_simulation(self, iterations: int = 5) -> list:
        """
        Run complete TASK3 simulation
        
        Args:
            iterations: Number of complete sensor readings to generate
            
        Returns:
            List of formatted output strings
        """
        outputs = []
        for _ in range(iterations):
            output = self.acquire_all_sensor_data()
            outputs.append(output)
            time.sleep(0.001)  # Simulate 10ms delay between samples
        
        return outputs


class SimulationDisplay:
    """Handles displaying simulation results"""
    
    @staticmethod
    def print_separator(title: str = ""):
        """Print a formatted separator line"""
        print("\n" + "=" * 70)
        if title:
            print(f"  {title}")
            print("=" * 70)
        else:
            print()
    
    @staticmethod
    def display_task_two_output(outputs: list):
        """Display TASK2 simulation results"""
        for output in outputs:
            value = output.strip()
            print(f"Sample : {value}")
    
#     @staticmethod
#     def display_task_three_output(outputs: list):
#         """Display TASK3 simulation results"""
#         SimulationDisplay.print_separator("LAB08_TASK3 - Gyroscope Full Data (Polling SPI)")
#         print("\nSimulation Output (Format: Temp, X-rotation, Y-rotation, Z-rotation):\n")
#         print(f"  {'Sample':<8} | {'Temp':>4} | {'X-Axis (dps)':>12} | {'Y-Axis (dps)':>12} | {'Z-Axis (dps)':>12}")
#         print("  " + "-" * 65)
        
#         for i, output in enumerate(outputs, 1):
#             parts = output.strip().split(',')
#             if len(parts) == 4:
#                 temp = parts[0]
#                 x_val = f"{float(parts[1]):>11.2f}"
#                 y_val = f"{float(parts[2]):>11.2f}"
#                 z_val = f"{float(parts[3]):>11.2f}"
#                 print(f"  {i:<8} | {temp:>4} | {x_val} | {y_val} | {z_val}")
    
#     @staticmethod
#     def display_task_three_output_extended(outputs: list):
#         """Display TASK3 simulation results with extended output showing pattern"""
#         SimulationDisplay.print_separator("LAB08_TASK3 - Gyroscope Full Data (Polling SPI - Extended)")
#         print("\nFull Simulation Output (Similar to Reference Graph):\n")
#         print(f"  {'Sample':>6} | {'Temp':>5} | {'X-Axis':>8} | {'Y-Axis':>8} | {'Z-Axis':>8}")
#         print("  " + "-" * 50)
        
#         for i, output in enumerate(outputs, 1):
#             parts = output.strip().split(',')
#             if len(parts) == 4:
#                 temp = parts[0]
#                 x_val = f"{float(parts[1]):>7.2f}"
#                 y_val = f"{float(parts[2]):>7.2f}"
#                 z_val = f"{float(parts[3]):>7.2f}"
#                 print(f"  {i:>6} | {temp:>5} | {x_val} | {y_val} | {z_val}")
    
#     @staticmethod
#     def display_program_info():
#         """Display information about the programs"""
#         SimulationDisplay.print_separator("Program Simulation Information")
        
#         print("\nPROGRAM DESCRIPTIONS:\n")
        
#         print("TASK 2 - Temperature Monitoring (Interrupt-Driven):")
#         print("  • Uses SPI with interrupt callbacks (Tx/Rx completion)")
#         print("  • Reads temperature from register 0x26")
#         print("  • Converts raw sensor value to temperature (formula: -value + 43)")
#         print("  • Outputs single temperature value every ~1 second")
#         print("  • Control Register: 0x20 = 0b00001111 (Power on, all axes)")
#         print("  • Temperature range: 20-22C (fluctuating)")
        
#         print("\nTASK 3 - Multi-Axis Gyroscope Monitoring (Polling):")
#         print("  • Uses SPI with polling (blocking calls)")
#         print("  • Reads temperature + X, Y, Z rotation simultaneously")
#         print("  • Converts raw counts to degrees per second (dps)")
#         print("  • Sensitivity: 0.00875 dps per count")
#         print("  • Outputs 4 values every ~10 milliseconds")
#         print("  • Control Register: 0x20 = 0b10001111 (Power on, 500dps range)")
#         print("  • Temperature steps: 12.5 (0-150) -> 11.5 (150-250) -> 10 (250+)")
        
#         print("\nVARIABLE REFACTORING:")
#         print("  Original          ->  Refactored")
#         print("  " + "-" * 35)
#         print("  temp_raw          ->  sensorDataByte")
#         print("  check             ->  spiTransferActive")
#         print("  temp              ->  transmitCmd")
#         print("  gyro_init()       ->  initializeGyroscope()")
#         print("  tmp()             ->  requestTemperatureSample()")
#         print("  gyro_read_reg()   ->  fetchRegisterValue()")
#         print("  gyro_read_xyz()   ->  acquireAxisData()")
#         print("  x, y, z           ->  xAxis, yAxis, zAxis")
#         print("  x_dps, y_dps, z_dps  ->  xRotation, yRotation, zRotation")
#         print("  buf               ->  serialBuffer")
    
    @staticmethod
    def plot_task3_data(outputs: list):
        """Plot TASK3 data like the reference graph"""
        # Parse the output data
        samples = []
        temps = []
        x_axis = []
        y_axis = []
        z_axis = []
        
        for i, output in enumerate(outputs, 1):
            parts = output.strip().split(',')
            if len(parts) == 4:
                samples.append(i)
                temps.append(int(parts[0]))
                x_axis.append(float(parts[1]))
                y_axis.append(float(parts[2]))
                z_axis.append(float(parts[3]))
        
        # Create the plot
        fig, ax = plt.subplots(figsize=(14, 6))
        
        # Plot temperature (magenta/pink)
        ax.plot(samples, temps, color='#FF00FF', linewidth=2.5, label='Temp (LSB)', marker='o', markersize=3)
        
        # Plot X, Y, Z rotations
        ax.plot(samples, x_axis, color='#FF0000', linewidth=1.2, label='X (dps)', alpha=0.8)
        ax.plot(samples, y_axis, color='#00AA00', linewidth=1.2, label='Y (dps)', alpha=0.8)
        ax.plot(samples, z_axis, color='#0000FF', linewidth=1.2, label='Z (dps)', alpha=0.8)
        
        # Formatting
        ax.set_xlabel('Time (samples)', fontsize=12)
        ax.set_ylabel('Value', fontsize=12)
        ax.set_title('Gyroscope Temperature & Angular Velocity', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        ax.set_ylim(-2.5, 14)
        
        # Save and show
        plt.tight_layout()
        plot_file = 'LAB08_TASK3_Gyroscope_Plot.png'
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"[PLOT SAVED] Graph saved to: {plot_file}")
        
        try:
            plt.show()
        except Exception as e:
            print(f"[INFO] Plot display skipped (non-GUI environment): {type(e).__name__}")
        finally:
            plt.close(fig)
        
        return plot_file


def main():
    """Main simulation entry point"""
    
    # Initialize gyroscope simulator for TASK2 with stable temperature fluctuation (20-22°C)
    sensor_task2 = GyroscopeSensorSimulator(base_temperature=20, simulation_mode="task2_stable")
    
    # Run TASK2 simulation
    task2 = TaskTwoSimulator(sensor_task2)
    task2_outputs = task2.run_simulation(iterations=9)  # Generate 9 samples to show fluctuation
    SimulationDisplay.display_task_two_output(task2_outputs)
    
    # Initialize gyroscope simulator for TASK3 with stepped temperature pattern
    sensor_task3 = GyroscopeSensorSimulator(base_temperature=12, simulation_mode="task3_stepped")
    
    # Run TASK3 simulation with more samples to show stepped pattern (400 to cover all temperature zones)
    task3 = TaskThreeSimulator(sensor_task3)
    task3_outputs = task3.run_simulation(iterations=400)  # 400 samples to show the complete stepping: 12.5 (0-150), 11.5 (150-250), 10 (250+)
    
    # Plot TASK3 data
    print("\n")
    plot_file = SimulationDisplay.plot_task3_data(task3_outputs)


if __name__ == "__main__":
    main()