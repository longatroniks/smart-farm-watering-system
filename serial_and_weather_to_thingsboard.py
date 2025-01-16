import serial
import json
import time
import requests
import re
import threading

# Serial port configuration
SERIAL_PORT = 'COM13'  # COM port
BAUD_RATE = 115200  # Baud rate for the serial communication

# ThingsBoard details
THINGSBOARD_HOST = "https://srv-iot.diatel.upm.es"  # ThingsBoard URL
WEATHER_ACCESS_TOKEN = "igxnbStVswAQD2dwgsAk"  # Weather Station device's Access Token
NODE_ACCESS_TOKEN = "8lpRi4s20USJgwDYKX0A"  # Node device's Access Token
WEATHER_THINGSBOARD_URL = f"{THINGSBOARD_HOST}/api/v1/{WEATHER_ACCESS_TOKEN}/telemetry"
NODE_THINGSBOARD_URL = f"{THINGSBOARD_HOST}/api/v1/{NODE_ACCESS_TOKEN}/telemetry"

# OpenWeatherAPI details
CITY = "Madrid"  # The city
API_KEY = "4de87bbcc11e4c53d3ab1fc04c4105d6"  # OpenWeatherAPI key
WEATHER_URL = f"https://api.openweathermap.org/data/2.5/forecast?q={CITY}&units=metric&appid={API_KEY}"

# Define the sleep interval (in seconds) for both threads
SLEEP_INTERVAL_API = 10  # The sleep interval for the weather API
SLEEP_INTERVAL_SERIAL = 5  # The sleep interval for the serial port

def read_from_mbed(port, baudrate=115200, timeout=1):
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Connected to {port} at {baudrate} baud.")

            while True:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Extract JSON part using regex
                    match = re.search(r'\{.*\}', line)
                    if match:
                        json_data = match.group(0)  # Extract the JSON string
                        try:
                            # Attempt to parse the extracted JSON
                            data = json.loads(json_data)
                            print("Received JSON:", data)
                            # Validate and adapt data format
                            formatted_data = format_data(data)
                            if formatted_data:
                                # Send the data to ThingsBoard
                                send_to_thingsboard(NODE_THINGSBOARD_URL, formatted_data)
                        except json.JSONDecodeError:
                            print("Error parsing JSON:", json_data)
                    else:
                        print("No valid JSON found in the line.")
    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")

def format_data(data):
    """
    Format the data into a dictionary suitable for ThingsBoard.
    This ensures all required keys are present and their types are correct.
    """
    try:
        formatted_data = {
            "brightness_sensor": int(data.get("brightness_sensor", 0)),
            "humidity_sensor": float(data.get("humidity_sensor", 0.0)),
            "moisture_sensor": int(data.get("moisture_sensor", 0)),
            "temperature_sensor": float(data.get("temperature_sensor", 0.0)),
            "pump_state": str(data.get("pump_state", "OFF")),
            "water_level_sensor": int(data.get("water_level_sensor", 0)),
        }
        return formatted_data
    except (ValueError, TypeError) as e:
        print(f"Error formatting data: {e}")
        return None

def send_to_thingsboard(url, data):
    """Send telemetry data to ThingsBoard."""
    try:
        response = requests.post(url, json=data, timeout=10)
        if response.status_code == 200:
            print("Data sent to ThingsBoard successfully.")
        else:
            print(f"Failed to send data. Status code: {response.status_code}, Response: {response.text}")
    except requests.RequestException as e:
        print(f"Error sending data to ThingsBoard: {e}")

def fetch_weather_data():
    """Fetch weather data from OpenWeatherAPI and send to ThingsBoard."""
    try:
        response = requests.get(WEATHER_URL)
        
        if response.status_code == 200:
            data = response.json()
            # Check if data contains the expected keys
            if "list" in data and len(data["list"]) > 0:
                # Extract relevant data
                rain_probability = data["list"][0]["pop"] * 100  # "pop" is the probability of rain
                telemetry_data = {
                    "pressure": data["list"][0]["main"]["pressure"],
                    "wind_speed": data["list"][0]["wind"]["speed"],
                    "rain_probability": rain_probability
                }
                send_to_thingsboard(WEATHER_THINGSBOARD_URL, telemetry_data)
            else:
                print("Weather data missing 'list' or it's empty")
        else:
            print(f"Failed to fetch weather data: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Error fetching weather data: {e}")

def weather_thread():
    """Fetch weather data in a loop every SLEEP_INTERVAL seconds."""
    while True:
        fetch_weather_data()
        time.sleep(SLEEP_INTERVAL_API)  # Wait for the sleep interval before fetching again

def serial_thread():
    """Read from the mbed serial in a loop with a SLEEP_INTERVAL."""
    while True:
        read_from_mbed(SERIAL_PORT, baudrate=BAUD_RATE)
        time.sleep(SLEEP_INTERVAL_SERIAL)  # Wait for the sleep interval before reading from the serial port again

if __name__ == "__main__":
    # Start weather API fetch thread
    weather_thread_instance = threading.Thread(target=weather_thread, daemon=True)
    weather_thread_instance.start()

    # Start serial reading thread
    serial_thread_instance = threading.Thread(target=serial_thread, daemon=True)
    serial_thread_instance.start()

    # Main thread waits for both threads to continue running
    while True:
        time.sleep(1)
        