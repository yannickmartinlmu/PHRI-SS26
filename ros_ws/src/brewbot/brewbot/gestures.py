#!/usr/bin/env python3
"""The three gestures and the topic they arrive on — one namespace, two nodes.

Both recognizers (arm camera + USB camera) publish into the same
`combined_camera` output_namespace, so this one topic is every hand the robot
can see. See config/*_gestures.yaml.

The STATE topic, not `gesture_event`: the event only fires on a transition, so a
hand already raised while the arm mimes and swings round produces no edge at all
and the wait runs its full timeout. This one republishes the held gesture at
10 Hz, so it is seen the moment the camera arrives, and a dropped message fixes
itself on the next tick. Price: it also ticks "none", and it is TRANSIENT_LOCAL,
so every subscriber MUST drop "none" — see NONE. The stale-thumb worry that
argued for the event topic is handled by the recognizer's lost_timeout_sec: the
state expires 0.4 s after the hand goes down, so there is nothing old to latch.

The recognizer maps everything else in MediaPipe's canned model to "none", so
after the NONE guard every message here is one of these three.
"""

TOPIC = "/brewbot/perception/combined_camera/gesture"

NONE = "none"         # no hand, or a gesture we do not act on — always ignore
UP = "thumbs_up"      # yes / bring this one
DOWN = "thumbs_down"  # no / offer the next one
PALM = "open_palm"    # stop — stop talking, stop offering
