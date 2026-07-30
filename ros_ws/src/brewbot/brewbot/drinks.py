#!/usr/bin/env python3
"""The menu — one dict, four nodes.

Key   = the word a human says. Also what SuggestDrink/BringDrink carry, what the
        estimator suggests, and what the reply classifier accepts. One namespace.
Value = the coffee_machine_actuator beverage, or None if it comes from the sink.

Everything in here is actually makeable end to end. Adding a key that the arm
cannot route or the machine cannot brew is what used to make "tea" silently hand
the glass back, so this table is the only place a drink may be introduced.
"""

MENU = {
    "water": None,        # cold, from the tap — the user fills it themselves
    "coffee": "coffee",
    "tea": "tea_water",   # same machine as coffee: hot water at 70 C, bag is manual
    # Hot milk belongs here the day a milk-froth program id is confirmed
    # on the EQ700 over Home Connect. One line, no other file changes.
}
