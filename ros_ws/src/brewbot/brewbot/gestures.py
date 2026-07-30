#!/usr/bin/env python3
"""The three gestures and the topic they arrive on — one namespace, two nodes.

Both recognizers (arm camera + USB camera) publish into the same
`combined_camera` output_namespace, so this one topic is every hand the robot
can see. See config/*_gestures.yaml.

EVENT_TOPIC, not `<ns>/gesture`: `/gesture` is a TRANSIENT_LOCAL *state* stream
that republishes the currently held gesture — including "none" — every tick from
both recognizers. Subscribing to that makes every wait answer itself instantly
with "none", and hands a fresh subscriber a thumbs-up from two minutes ago.
`gesture_event` is VOLATILE and fires once, when a new gesture is confirmed.

The recognizer maps everything else in MediaPipe's canned model to "none" and
drops it, so every event that arrives here is one of these three.
"""

EVENT_TOPIC = "/brewbot/perception/combined_camera/gesture_event"

UP = "thumbs_up"      # yes / bring this one
DOWN = "thumbs_down"  # no / offer the next one
PALM = "open_palm"    # stop — stop talking, stop offering
