#!/usr/bin/env python3
"""
Data Analysis Script for Precision Farming Robot

Analyzes sensor data from CSV files and generates insights.
"""

import sys
import csv
from statistics import mean, stdev

def analyze_data(filename):
    """Analyze sensor data from CSV file."""
    
    print(f"\n{'='*60}")
    print(f"Precision Farming Robot - Data Analysis")
    print(f"{'='*60}\n")
    print(f"Analyzing: {filename}\n")
    
    temperatures = []
    humidities = []
    moistures = []
    distances = []
    
    try:
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                try:
                    temperatures.append(float(row['Temperature']))
                    humidities.append(float(row['Humidity']))
                    moistures.append(int(row['SoilMoisture']))
                    distances.append(int(row['Distance']))
                except (ValueError, KeyError) as e:
                    print(f"Warning: Skipping invalid row: {e}")
                    continue
        
        if not temperatures:
            print("Error: No valid data found in file!")
            return
        
        # Calculate statistics
        print("📊 Statistical Summary")
        print("-" * 60)
        
        print("\n🌡️  Temperature (°C)")
        print(f"   Average: {mean(temperatures):.2f}")
        print(f"   Min:     {min(temperatures):.2f}")
        print(f"   Max:     {max(temperatures):.2f}")
        if len(temperatures) > 1:
            print(f"   Std Dev: {stdev(temperatures):.2f}")
        
        print("\n💧 Humidity (%)")
        print(f"   Average: {mean(humidities):.2f}")
        print(f"   Min:     {min(humidities):.2f}")
        print(f"   Max:     {max(humidities):.2f}")
        if len(humidities) > 1:
            print(f"   Std Dev: {stdev(humidities):.2f}")
        
        print("\n🌱 Soil Moisture")
        print(f"   Average: {mean(moistures):.0f}")
        print(f"   Min:     {min(moistures)}")
        print(f"   Max:     {max(moistures)}")
        if len(moistures) > 1:
            print(f"   Std Dev: {stdev(moistures):.2f}")
        
        # Convert to percentage (assuming dry=1023, wet=350)
        moisture_pct = [(1023 - m) / (1023 - 350) * 100 for m in moistures]
        print(f"   Average: {mean(moisture_pct):.1f}%")
        
        print("\n📏 Distance (cm)")
        print(f"   Average: {mean(distances):.2f}")
        print(f"   Min:     {min(distances)}")
        print(f"   Max:     {max(distances)}")
        if len(distances) > 1:
            print(f"   Std Dev: {stdev(distances):.2f}")
        
        # Insights
        print("\n" + "="*60)
        print("🔍 Insights")
        print("-" * 60)
        
        avg_temp = mean(temperatures)
        if avg_temp < 15:
            print("❄️  Temperature is LOW - crops may be stressed")
        elif avg_temp > 30:
            print("🔥 Temperature is HIGH - increase watering frequency")
        else:
            print("✅ Temperature is in optimal range")
        
        avg_humidity = mean(humidities)
        if avg_humidity < 40:
            print("🏜️  Humidity is LOW - consider misting")
        elif avg_humidity > 70:
            print("💦 Humidity is HIGH - ensure good ventilation")
        else:
            print("✅ Humidity is in optimal range")
        
        avg_moisture_pct = mean(moisture_pct)
        if avg_moisture_pct < 30:
            print("🚰 Soil is DRY - irrigation needed")
        elif avg_moisture_pct > 70:
            print("💧 Soil is WET - reduce watering")
        else:
            print("✅ Soil moisture is optimal")
        
        # Count obstacles
        obstacle_count = sum(1 for d in distances if d < 30 and d > 0)
        print(f"\n🚧 Obstacles encountered: {obstacle_count} times")
        
        print("\n" + "="*60)
        print(f"Total data points: {len(temperatures)}")
        print("="*60 + "\n")
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found!")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_data.py <data_file.csv>")
        print("\nExample: python analyze_data.py data/sample_data.csv")
        sys.exit(1)
    
    analyze_data(sys.argv[1])
