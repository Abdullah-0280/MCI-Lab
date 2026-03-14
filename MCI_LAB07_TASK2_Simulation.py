#!/usr/bin/env python3
"""
Simulation of MCI_LAB07_TASK2: Live ADC & Filtered Data
Simulates ADC data with rolling average filter
Generates plot matching the reference image
"""

import numpy as np
import matplotlib.pyplot as plt
import random
from collections import deque


class ADCDataSimulator:
    """Generates realistic ADC sensor data"""
    
    def __init__(self, base_value=2300, noise_amplitude=600):
        """
        Initialize ADC simulator
        
        Args:
            base_value: Center/nominal ADC value
            noise_amplitude: Amplitude of noise oscillations
        """
        self.base_value = base_value
        self.noise_amplitude = noise_amplitude
        self.time_step = 0
        
    def generate_sample(self):
        """Generate next ADC sample with realistic noise"""
        # Create high-frequency oscillating pattern with random variations
        oscillation = self.noise_amplitude * np.sin(self.time_step * 0.08)
        random_noise = random.gauss(0, 80)
        sample = self.base_value + oscillation + random_noise
        
        # Keep within realistic ADC bounds (0-4095 for 12-bit ADC)
        sample = max(1500, min(3500, sample))
        self.time_step += 1
        return int(sample)


class RollingAverageFilter:
    """Implements a rolling average filter"""
    
    def __init__(self, window_size=10):
        """
        Initialize filter
        
        Args:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        
    def filter_sample(self, new_value):
        """
        Add new sample and compute average
        
        Args:
            new_value: New ADC sample
            
        Returns:
            Current filtered (averaged) value
        """
        self.buffer.append(new_value)
        if len(self.buffer) == 0:
            return new_value
        return sum(self.buffer) / len(self.buffer)


class ADCSimulationDisplay:
    """Handles display and plotting of ADC data"""
    
    @staticmethod
    def print_raw_data(raw_samples, filtered_samples):
        """Print raw and filtered data"""
        print("\n=== ADC Output Data ===")
        print("Time(ms)\tRaw ADC\tFiltered Mean")
        print("-" * 40)
        for i, (raw, filt) in enumerate(zip(raw_samples, filtered_samples)):
            time_ms = 5410 + i * 2
            print(f"{time_ms}\t{int(raw)}\t{int(filt):.0f}")
    
    @staticmethod
    def generate_plot(raw_data, filtered_data, output_file="ADC_Filter_Simulation.png"):
        """
        Generate plot matching reference image style with high-density dataset
        
        Args:
            raw_data: List of raw ADC values
            filtered_data: List of filtered values
            output_file: Output PNG filename
        """
        # Generate time axis: 15000 samples over 500ms (5400-5900ms range)
        # This creates sampling rate of 30 kHz
        time_start = 5400
        time_end = 5900
        time_axis = np.linspace(time_start, time_end, len(raw_data))
        
        # Create figure
        fig, ax = plt.subplots(figsize=(14, 7))
        
        # Plot raw data (red) with noise - very tight data points
        ax.plot(time_axis, raw_data, color='#CC0000', linewidth=0.8, 
                label='Raw ADC', alpha=0.85, marker='', markersize=0)
        
        # Plot filtered data (blue) smoothed line
        ax.plot(time_axis, filtered_data, color='#0000FF', linewidth=3.2,
                label='Filtered Mean', alpha=0.95, marker='', markersize=0)
        
        # Formatting to match reference image
        ax.set_xlabel('Time (ms)', fontsize=12, fontweight='bold')
        ax.set_ylabel('ADC Value', fontsize=12, fontweight='bold')
        ax.set_title('Live ADC & Filtered Data', fontsize=14, fontweight='bold')
        
        # Set axis limits to match reference exactly
        ax.set_xlim(5400, 5900)
        ax.set_ylim(0, 5000)
        
        # Grid and legend matching reference image
        ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5, color='gray')
        ax.legend(loc='upper right', fontsize=11, framealpha=0.95, 
                 edgecolor='black', fancybox=False)
        
        # Set tick parameters for cleaner appearance
        ax.tick_params(axis='both', which='major', labelsize=10)
        
        # Improve layout
        plt.tight_layout()
        
        # Save figure with high quality
        plt.savefig(output_file, dpi=150, bbox_inches='tight', facecolor='white')
        print(f"\n[PLOT SAVED] Graph saved to: {output_file}")
        
        try:
            plt.show()
        except Exception as e:
            print(f"[INFO] Plot display skipped (non-GUI environment): {type(e).__name__}")
        finally:
            plt.close(fig)
        
        return output_file


def main():
    """Main simulation execution"""
    
    print("=" * 60)
    print("MCI_LAB07_TASK2: ADC Data with Rolling Average Filter")
    print("Simulation Parameters:")
    print("  - ADC Center Value: 2300")
    print("  - Noise Amplitude: 600")
    print("  - Filter Window Size: 10")
    print("  - Samples Generated: 15000 (high-resolution)")
    print("=" * 60)
    
    # Initialize simulator and filter
    adc_sim = ADCDataSimulator(base_value=2300, noise_amplitude=600)
    rolling_filter = RollingAverageFilter(window_size=10)
    
    # Generate large dataset for dense plot
    raw_adc_data = []
    filtered_adc_data = []
    
    num_samples = 15000  # Large dataset to show dense noise pattern
    
    for _ in range(num_samples):
        # Generate raw ADC sample
        raw_sample = adc_sim.generate_sample()
        raw_adc_data.append(raw_sample)
        
        # Apply filter
        filtered_sample = rolling_filter.filter_sample(raw_sample)
        filtered_adc_data.append(filtered_sample)
    
    # Display results
    print(f"\nGenerated {num_samples} samples")
    print(f"Raw ADC Range: {min(raw_adc_data)} - {max(raw_adc_data)}")
    print(f"Filtered Range: {min(filtered_adc_data):.0f} - {max(filtered_adc_data):.0f}")
    
    # Print sample data
    ADCSimulationDisplay.print_raw_data(raw_adc_data[:20], filtered_adc_data[:20])
    
    # Generate plot
    print("\nGenerating high-resolution plot...")
    ADCSimulationDisplay.generate_plot(raw_adc_data, filtered_adc_data)
    
    print("\n✓ Simulation complete!")
    print("✓ ADC sensor data successfully simulated")
    print("✓ Rolling average filter applied")
    print("✓ High-density plot generated matching reference image")


if __name__ == "__main__":
    main()
