#!/usr/bin/env python3
"""Coffee machine actuation layer backed by Home Assistant

Accepts actuation requests from orchestration or the arm controller,
and translates them into Home Connect service calls.
"""

import sys

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from brewbot.drinks import MENU
from brewbot.home_assistant import HomeAssistantClient, HomeAssistantError
from brewbot_interfaces.action import DispenseDrink

DEVICE_ID = "4b7848243ac398e168e7d08c3a80a8c8"
OPERATION_STATE_ENTITY = "sensor.coffee_maker_operation_state"
DISPENSE_REQUEST_TOPIC = "/coffee_machine/dispense_request"

# Home Connect programs, keyed by the actuator's OWN beverage names — the human
# menu lives in brewbot/drinks.py and maps into these keys. Same machine dispenses
# all of them; tea_water is the hot_water program at a lower temperature.
PROGRAMS = {
    "coffee": "consumer_products_coffee_maker_program_beverage_coffee",
    # "espresso": "consumer_products_coffee_maker_program_beverage_espresso",
    "hot_water": "consumer_products_coffee_maker_program_beverage_hot_water",
    "tea_water": "consumer_products_coffee_maker_program_beverage_hot_water",
}

# The cross-file contract, checked at import: nothing offerable is unbrewable.
assert set(MENU.values()) <= set(PROGRAMS) | {None}, \
    f"MENU beverages missing from PROGRAMS: {set(MENU.values()) - set(PROGRAMS) - {None}}"

OPTIONS = {
    "fill_quantity": "consumer_products_coffee_maker_option_fill_quantity",
    "hot_water_temperature": "consumer_products_coffee_maker_option_hot_water_temperature",
}

FILL_QUANTITIES_ML = {100, 150, 200, 250}
HOT_WATER_TEMPERATURES = {
    #"consumer_products_coffee_maker_enum_type_hot_water_temperature_white_tea",
    #"consumer_products_coffee_maker_enum_type_hot_water_temperature_green_tea",
    #"consumer_products_coffee_maker_enum_type_hot_water_temperature_black_tea",
    "consumer_products_coffee_maker_enum_type_hot_water_temperature_70_c",
    "consumer_products_coffee_maker_enum_type_hot_water_temperature_75_c",
    "consumer_products_coffee_maker_enum_type_hot_water_temperature_80_c",
    "consumer_products_coffee_maker_enum_type_hot_water_temperature_85_c",
    "consumer_products_coffee_maker_enum_type_hot_water_temperature_90_c",
}
HOT_WATER_BEVERAGES = {"hot_water", "tea_water"}
DEFAULT_BEVERAGE_TEMPERATURES = {
    "tea_water": "consumer_products_coffee_maker_enum_type_hot_water_temperature_70_c",
    "hot_water": "consumer_products_coffee_maker_enum_type_hot_water_temperature_90_c",
}

READY_STATES = {"ready", "inactive", "standby", "idle"}
BUSY_STATES = {"run", "pause", "delayedstart", "actionrequired"}


class CoffeeMachineActuator(Node):
    def __init__(self):
        super().__init__("coffee_machine_actuator")
        cb = ReentrantCallbackGroup()

        self.declare_parameter("device_id", DEVICE_ID)
        self.declare_parameter("operation_state_entity", OPERATION_STATE_ENTITY)
        self.declare_parameter("request_timeout", 10.0)
        self.declare_parameter("require_ready_state", False)
        self.declare_parameter("dispense_request_topic", DISPENSE_REQUEST_TOPIC)
        self.declare_parameter("affects_to", "active_program")
        self.declare_parameter("fill_quantity_ml", 100)
        self.declare_parameter("tea_water_temperature", "")
        self.declare_parameter("hot_water_temperature", "")

        self._device_id = self.get_parameter("device_id").value
        self._operation_state_entity = self.get_parameter("operation_state_entity").value
        self._require_ready_state = self.get_parameter("require_ready_state").value
        self._affects_to = self.get_parameter("affects_to").value
        timeout = self.get_parameter("request_timeout").value
        topic = self.get_parameter("dispense_request_topic").value
        self._ha = HomeAssistantClient(timeout=timeout)
        self._busy = False
        self._dispense_sub = self.create_subscription(
            String,
            topic,
            self._on_dispense_request,
            10,
            callback_group=cb,
        )

        self._server = ActionServer(
            self,
            DispenseDrink,
            "dispense_drink",
            self._execute,
            goal_callback=self._on_goal,
            callback_group=cb,
        )
        self.get_logger().info("Coffee machine actuator ready")

    def _on_dispense_request(self, msg):
        beverage = self._normalize_beverage(msg.data)
        if beverage not in PROGRAMS:
            self.get_logger().error(
                f"[coffee_topic] unsupported beverage: {msg.data}"
            )
            return
        if self._busy:
            self.get_logger().warn("[coffee_topic] busy - ignoring request")
            return
        self._busy = True
        try:
            self._start_checked_program(beverage)
            self.get_logger().info(f"[coffee_topic] started {beverage}")
        except Exception as exc:
            self.get_logger().error(f"[coffee_topic] failed: {exc}")
        finally:
            self._busy = False

    def _on_goal(self, goal_request):
        beverage = self._normalize_beverage(goal_request.beverage)
        if beverage not in PROGRAMS:
            self.get_logger().warn(
                f"[dispense_drink] unsupported beverage: "
                f"{goal_request.beverage}"
            )
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn("[dispense_drink] busy - rejecting goal")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        beverage = self._normalize_beverage(goal_handle.request.beverage)
        reason = goal_handle.request.reason.strip()
        feedback = DispenseDrink.Feedback(status="checking_machine")
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(
            f"[dispense_drink] beverage={beverage} reason={reason or 'unspecified'}"
        )
        try:
            goal_handle.publish_feedback(
                DispenseDrink.Feedback(status="starting_program")
            )
            self._start_checked_program(beverage)
            goal_handle.succeed()
            return DispenseDrink.Result(success=True, status=f"started {beverage}")
        except Exception as exc:
            self.get_logger().error(f"[dispense_drink] failed: {exc}")
            goal_handle.abort()
            return DispenseDrink.Result(success=False, status=str(exc))
        finally:
            self._busy = False

    def _start_checked_program(self, beverage):
        state = self.get_operation_state()
        if self._machine_busy(state):
            raise HomeAssistantError(
                f"coffee machine is busy: operation_state={state}"
            )
        if self._require_ready_state and not self._machine_ready(state):
            raise HomeAssistantError(
                f"coffee machine is not ready: operation_state={state}"
            )
        self.start_program(beverage)

    def get_operation_state(self):
        state = self._ha.get_state(self._operation_state_entity)
        if not isinstance(state, dict):
            return "unknown"
        return str(state.get("state", "unknown")).lower()

    def start_program(self, beverage, options=None):
        beverage = self._normalize_beverage(beverage)
        body = {
            "device_id": self._device_id,
            "affects_to": self._affects_to,
            "program": PROGRAMS[beverage],
        }
        body.update(self._read_configured_options(beverage))
        if options:
            body.update(options)
        self.get_logger().info(f"[home_connect] set_program_and_options: {body}")
        return self._ha.call_service("home_connect", "set_program_and_options", body)

    def _read_configured_options(self, beverage):
        configured = {}

        fill_quantity = int(self.get_parameter("fill_quantity_ml").value)
        if fill_quantity not in FILL_QUANTITIES_ML:
            raise HomeAssistantError(
                f"fill_quantity_ml must be one of {sorted(FILL_QUANTITIES_ML)}"
            )
        configured[OPTIONS["fill_quantity"]] = fill_quantity

        if beverage in HOT_WATER_BEVERAGES:
            temperature = self._temperature_for(beverage)
            if temperature not in HOT_WATER_TEMPERATURES:
                raise HomeAssistantError(
                    f"{beverage} temperature is not supported"
                )
            configured[OPTIONS["hot_water_temperature"]] = temperature

        return configured

    def _temperature_for(self, beverage):
        parameter_name = f"{beverage}_temperature"
        configured = self.get_parameter(parameter_name).value
        if configured:
            return configured
        return DEFAULT_BEVERAGE_TEMPERATURES[beverage]

    @staticmethod
    def _normalize_beverage(beverage):
        return beverage.strip().lower().replace("-", "_").replace(" ", "_")

    @staticmethod
    def _machine_busy(state):
        return str(state).lower() in BUSY_STATES

    @staticmethod
    def _machine_ready(state):
        return str(state).lower() in READY_STATES


def main():
    rclpy.init()
    try:
        node = CoffeeMachineActuator()
    except HomeAssistantError as exc:
        print(f"coffee_machine_actuator: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
