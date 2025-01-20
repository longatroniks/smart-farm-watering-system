import requests
import time
import json

# ThingsBoard configuration
THINGSBOARD_HOST = "https://srv-iot.diatel.upm.es"
WEATHER_ACCESS_TOKEN = "igxnbStVswAQD2dwgsAk"
WEATHER_THINGSBOARD_URL = f"{THINGSBOARD_HOST}/api/v1/{WEATHER_ACCESS_TOKEN}/telemetry"

# OpenWeatherAPI configuration
CITY = "Madrid"
API_KEY = "4de87bbcc11e4c53d3ab1fc04c4105d6"
WEATHER_URL = f"https://api.openweathermap.org/data/2.5/forecast?q={CITY}&units=metric&appid={API_KEY}"

# Update interval (in seconds)
UPDATE_INTERVAL = 10

def send_to_thingsboard(data):
    """Send weather data to ThingsBoard."""
    try:
        response = requests.post(WEATHER_THINGSBOARD_URL, json=data, timeout=10)
        if response.status_code == 200:
            print("Weather data sent to ThingsBoard successfully.")
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
            if "list" in data and len(data["list"]) > 0:
                # Extract weather data from first forecast entry
                current_weather = data["list"][0]
                
                # Format telemetry data
                telemetry_data = {
                    "pressure": current_weather["main"]["pressure"],
                    "wind_speed": current_weather["wind"]["speed"],
                    "rain_probability": current_weather["pop"] * 100  # Convert to percentage
                }
                
                print(f"Weather data fetched: {json.dumps(telemetry_data, indent=2)}")
                send_to_thingsboard(telemetry_data)
            else:
                print("Weather data missing 'list' or it's empty")
        else:
            print(f"Failed to fetch weather data: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Error fetching weather data: {e}")

def main():
    print(f"Starting weather monitoring for {CITY}")
    print(f"Update interval: {UPDATE_INTERVAL} seconds")
    
    while True:
        fetch_weather_data()
        time.sleep(UPDATE_INTERVAL)

if __name__ == "__main__":
    main()