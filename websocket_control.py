import json
import asyncio
import websockets
import rclpy
from hello_helpers.hello_misc import HelloNode

class WebsocketStretchControl(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        
        # Define the  input range
        self.input_min = 0.0
        self.input_max = 100.0
        
        # Define the Stretch robot's lift physical limits (in meters)
        self.lift_min = 0.2
        self.lift_max = 1.0

    def map_value(self, val):
        # Clamp the input so command doesnt go out of bounds
        val = max(self.input_min, min(self.input_max, val))
        
        # Linear interpolation
        mapped_lift = self.lift_min + (val - self.input_min) * (self.lift_max - self.lift_min) / (self.input_max - self.input_min)
        return mapped_lift

    async def ws_handler(self, websocket, path=""):
        self.get_logger().info("WebSocket Client connected")
        
        async for message in websocket:
            try:
                # Parse the incoming data structure
                data = json.loads(message)
                
                # Check if our expected key is in the payload
                if 'lift_input' in data:
                    raw_val = float(data['lift_input'])
                    target_lift = self.map_value(raw_val)
                    
                    self.get_logger().info(f"Received Input: {raw_val} | Mapped Lift Command: {target_lift:.3f}m")
                    
                    # SEND COMMAND TO ROBOT:
                    # blocking=False ensures we immediately return to listen to the WebSocket stream
                    self.move_to_pose({'joint_lift': target_lift}, blocking=False)
                    
            except json.JSONDecodeError:
                self.get_logger().warning("Received malformed JSON.")
            except Exception as e:
                self.get_logger().error(f"Error processing message: {e}")

    def main(self):
        # Initialize the HelloNode architecture (starts the ROS spinner in a background thread)
        HelloNode.main(self, 'ws_stretch_control', 'ws_stretch_control')
        
        # Safely stow the robot upon startup
        self.stow_the_robot()


if __name__ == '__main__':
    # 1. Start the ROS 2 node
    node = WebsocketStretchControl()
    node.main()
    
    print("Starting WebSocket Server on ws://0.0.0.0:8765...")
    
    # 2. Start the WebSocket server on the main thread
    loop = asyncio.get_event_loop()
    start_server = websockets.serve(node.ws_handler, "0.0.0.0", 8765)
    loop.run_until_complete(start_server)
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print("\nShutting down WebSocket server and ROS node...")
    finally:
        # Ensures the HelloNode ROS thread exits cleanly
        node.new_thread.join()
