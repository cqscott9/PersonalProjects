import asyncio
import websockets
import json

async def send_test_command():
    # Replace with your Stretch robot's actual IP address
    uri = "ws://<ROBOT_IP>:8765" 

    print(f"Connecting to {uri}...")
    async with websockets.connect(uri) as websocket:
        # 50.0 is exactly halfway through your 0.0 - 100.0 input range.
        # This should map to 0.6 meters on the lift (halfway between 0.2 and 1.0).
        test_data = {"lift_input": 50.0}
        payload = json.dumps(test_data)

        print(f"Sending payload: {payload}")
        await websocket.send(payload)
        print("Command sent successfully!")

if __name__ == "__main__":
    asyncio.run(send_test_command())