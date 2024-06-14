import subprocess
import requests
import json
import asyncio
import websockets
import time

def start_ngrok():
    print("Starting ngrok...")
    try:
        ngrok_process = subprocess.Popen(['gnome-terminal', '--', 'ngrok', 'tcp', '9090'],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception as e:
        print(f"Error starting ngrok process: {e}")
        return None

    return ngrok_process

def get_ngrok_url():
    time.sleep(5)  # Wait for ngrok to start (adjust delay as needed)
    try:
        response = requests.get('http://localhost:4040/api/tunnels')
        response.raise_for_status()
        data = response.json()
        tunnels = data.get('tunnels', [])
        for tunnel in tunnels:
            if tunnel.get('proto') == 'tcp':
                ngrok_url = tunnel.get('public_url')
                print(f"Ngrok public URL: {ngrok_url}")
                return ngrok_url
        print("No TCP tunnel found in Ngrok response")
    except requests.RequestException as e:
        print(f"Error during Ngrok request: {e}")
    except ValueError as e:
        print(f"Error decoding Ngrok response JSON: {e}")
    return None

async def send_ngrok_url(ngrok_url):
    if ngrok_url:
        print(f"Sending ngrok URL to WebSocket server: {ngrok_url}")
        try:
            async with websockets.connect('wss://sans-agv.azurewebsites.net/ws/lidar') as websocket:
                port = ngrok_url.split(":")[-1]
                await websocket.send(json.dumps({"port": port}))
                response = await websocket.recv()
                print(f"Response from WebSocket server: {response}")
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"Connection closed with error: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")
    else:
        print("Failed to get ngrok URL")

async def main():
    ngrok_process = start_ngrok()
    if ngrok_process:
        ngrok_url = get_ngrok_url()
        await send_ngrok_url(ngrok_url)
        ngrok_process.terminate()
        print("Ngrok process terminated")

if __name__ == "__main__":
    print("Starting main function...")
    asyncio.run(main())
    print("Script execution completed")