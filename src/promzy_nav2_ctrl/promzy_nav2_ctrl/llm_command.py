#!/usr/bin/env python3
import os, json, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf_transformations

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
import openai  # Import the OpenAI SDK

# ---------- Places DB: load from installed share; fallback to source tree ----------
PLACE_DB = {}
try:
    pkg_share = get_package_share_directory('promzy_nav2_ctrl')
    p = Path(pkg_share) / 'config' / 'places.yaml'
    if not p.exists():
        # dev run-from-source fallback
        p = Path(__file__).resolve().parents[1] / 'config' / 'places.yaml'
    if p.exists():
        with open(p, 'r') as f:
            PLACE_DB = yaml.safe_load(f).get('places', {}) or {}
    else:
        print(f"[LLMNavBridge] ⚠️ places.yaml not found at {p}. Continuing with empty PLACE_DB")
except Exception as e:
    print(f"[LLMNavBridge] ⚠️ Failed to load places.yaml: {e}. Continuing with empty PLACE_DB")
    PLACE_DB = {}

# ---------- Azure client (OpenAI SDK) ----------
# Azure API setup for GPT-5 Model
endpoint = os.environ.get("AZURE_AI_ENDPOINT")  # Azure Endpoint
api_key = os.environ.get("AZURE_AI_KEY")  # Azure API Key
deployment_name = os.environ.get("AZURE_AI_MODEL")  # Deployment name (e.g., "gpt-5-chat")

if not all([endpoint, api_key, deployment_name]):
    print("Error: Ensure that AZURE_AI_ENDPOINT, AZURE_AI_KEY, and AZURE_AI_MODEL are set.")
    exit(1)

# Initialize the OpenAI client
openai.api_base = endpoint
openai.api_key = api_key

SYSTEM_PROMPT = """You are a navigation planner for a ROS2 robot.
Return ONLY valid JSON matching this schema:
{
  "action": "go_to" | "follow_waypoints" | "cancel" | "dock",
  "waypoints": [{"x": float, "y": float, "yaw": float}] | [],
  "options": {"frame_id": "map", "speed_limit": float|null}
}
Rules:
- If user mentions place names (e.g., 'kitchen', 'charging dock'), use tool:resolve_place to get coordinates.
- Prefer 'follow_waypoints' when user lists multiple stops or says 'then', 'after', 'and'.
- yaw is heading (radians). If unspecified, infer doorway-facing or default 0.0.
- Keep output terse. Do NOT include commentary.
"""

def pose_from_xyyaw(navigator: BasicNavigator, x, y, yaw, frame_id="map"):
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg

class LLMNavBridge(Node):
    def __init__(self):
        super().__init__('llm_nav_bridge')
        self.sub = self.create_subscription(String, '/nl_commands', self.on_command, 10)
        self.pub_status = self.create_publisher(String, '/nl_status', 10)

        # Nav2 commander
        self.navigator = BasicNavigator()
        init = pose_from_xyyaw(self.navigator, -2.0, -0.5, 0.0)
        self.navigator.setInitialPose(init)
        self.navigator.waitUntilNav2Active()

    # --- Tool: resolve a place name to (x,y,yaw) using PLACE_DB ---
    def tool_resolve_place(self, name: str):
        return PLACE_DB.get(name.strip().lower())

    def call_llm(self, user_text: str):
        tools = [{
            "type": "function",
            "function": {
                "name": "resolve_place",
                "description": "Resolve a named place to coordinates (x,y,yaw)",
                "parameters": {
                    "type": "object",
                    "properties": {"name": {"type": "string"}},
                    "required": ["name"]
                }
            }
        }]

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_text}
        ]

        # Request the LLM to generate structured output
        response = openai.ChatCompletion.create(
            model=deployment_name,
            messages=messages,
            temperature=0.2,
            max_tokens=300  # Adjust the token limit as needed
        )

        # Parse the response
        out = response.choices[0].message['content']
        print(f"Generated plan: {out}")  # Debugging the response

        try:
            plan = json.loads(out)
        except Exception as e:
            self.get_logger().error(f"Failed to parse LLM response: {e}")
            return {}

        # Resolve names → coordinates
        resolved = []
        for wp in plan.get("waypoints", []):
            print(f"Resolving waypoint: {wp}")  # Debugging each waypoint
            if isinstance(wp, dict):
                if all(k in wp for k in ("x", "y", "yaw")):
                    resolved.append(wp)
                elif "name" in wp:
                    coords = self.tool_resolve_place(wp["name"])
                    if coords: resolved.append(coords)
            elif isinstance(wp, str):
                coords = self.tool_resolve_place(wp)
                if coords: resolved.append(coords)

        plan["waypoints"] = resolved
        return plan

    def on_command(self, msg: String):
        text = msg.data.strip()
        print(f"Received command: {text}")  # Debugging

        self.pub_status.publish(String(data=f"👂 Received: {text}"))

        try:
            plan = self.call_llm(text)
            print(f"Generated plan: {plan}")  # Debugging the plan
        except Exception as e:
            self.get_logger().error(f"LLM error: {e}")
            self.pub_status.publish(String(data="❌ LLM error parsing command"))
            return

        action = plan.get("action", "follow_waypoints")
        waypoints = plan.get("waypoints", [])
        opts = plan.get("options", {}) or {}
        frame_id = opts.get("frame_id", "map")

        if action == "cancel":
            self.navigator.cancelTask()
            self.pub_status.publish(String(data="🛑 Cancelled current task."))
            return

        if not waypoints:
            self.pub_status.publish(String(data="⚠️ No waypoints resolved."))
            return

        poses = [pose_from_xyyaw(self.navigator, w["x"], w["y"], w.get("yaw", 0.0), frame_id) for w in waypoints]

        if action == "go_to" and poses:
            self.navigator.goToPose(poses[0])
        else:
            self.navigator.followWaypoints(poses)

        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb is not None and hasattr(fb, "current_waypoint"):
                self.pub_status.publish(String(data=f"🚗 Progress: waypoint {fb.current_waypoint}"))

        result = self.navigator.getResult()
        self.pub_status.publish(String(data=f"✅ Result: {result}"))

def main():
    rclpy.init()
    node = LLMNavBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
