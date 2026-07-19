#!/usr/bin/env python3

from flask import Flask
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from brewbot_interfaces.action import BringDrink


class DrinkWebNode(Node):

    def __init__(self):
        super().__init__("web_ui")
        self._action_client = ActionClient(self, BringDrink, 'bring_drink')

    def suggest_drink(self, drink):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Arm Controller not available")
            return None

        result_event = threading.Event()
        result_box = [None]

        def on_result(future):
            result_box[0] = future.result().result
            result_event.set()

        def on_goal(future):
            gh = future.result()
            if not gh.accepted:
                result_event.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        self._action_client.send_goal_async(
            BringDrink.Goal(drink=drink)
        ).add_done_callback(on_goal)

        result_event.wait(timeout=60.0)
        return result_box[0]


app = Flask(__name__)

drink_node = None


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>BrewBot</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      background: #1a1a2e;
      font-family: 'Segoe UI', sans-serif;
      color: #eee;
    }}
    h1 {{
      font-size: 2.5rem;
      margin-bottom: 0.25rem;
      letter-spacing: 2px;
    }}
    p.subtitle {{
      color: #888;
      margin-bottom: 3rem;
      font-size: 0.95rem;
    }}
    .buttons {{
      display: flex;
      gap: 1.5rem;
    }}
    .drink-btn {{
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 0.75rem;
      background: #16213e;
      border: 2px solid #0f3460;
      border-radius: 1.25rem;
      padding: 2rem 2.5rem;
      cursor: pointer;
      transition: transform 0.15s, border-color 0.15s, background 0.15s;
      font-size: 1rem;
      color: #eee;
    }}
    .drink-btn:hover {{
      transform: translateY(-4px);
      border-color: #e94560;
      background: #1f2d50;
    }}
    .drink-btn:active {{ transform: translateY(0); }}
    .drink-btn .icon {{ font-size: 3rem; }}
    .toast {{
      margin-top: 2.5rem;
      padding: 0.6rem 1.5rem;
      border-radius: 2rem;
      background: #0f3460;
      font-size: 0.9rem;
      opacity: {opacity};
      transition: opacity 0.4s;
    }}
  </style>
</head>
<body>
  <h1>BrewBot</h1>
  <p class="subtitle">What can I get you?</p>
  <div class="buttons">
    <form action="/drink/water" method="post">
      <button class="drink-btn" type="submit">
        <span class="icon">💧</span>Water
      </button>
    </form>
    <form action="/drink/coffee" method="post">
      <button class="drink-btn" type="submit">
        <span class="icon">☕</span>Coffee
      </button>
    </form>
  </div>
  <div class="toast">{message}</div>
</body>
</html>"""


@app.route("/")
def index():
    return PAGE.format(message="", opacity=0)


@app.route("/drink/<drink>", methods=["POST"])
def select_drink(drink):
    if drink not in ["water", "coffee"]:
        return PAGE.format(message="Unknown drink.", opacity=1)

    result = drink_node.suggest_drink(drink)

    if result is None:
        message = "Suggestion handler not available."
    elif result.success:
        message = f"On its way!"
    else:
        message = f"Maybe next time."

    return PAGE.format(message=message, opacity=1)


def ros_spin():
    rclpy.spin(drink_node)


def main():
    global drink_node

    rclpy.init()

    drink_node = DrinkWebNode()

    ros_thread = threading.Thread(
        target=ros_spin,
        daemon=True
    )
    ros_thread.start()

    try:
        app.run(
            host="0.0.0.0",  # accessible on LAN
            port=5151
        )
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        drink_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
