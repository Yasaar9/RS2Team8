#!/usr/bin/env python3
"""
RS2 Team 8 - Tour Guide Robot
Subsystem 3 & 4: Interaction, Execution and User Experience

Changes from UI_final_version:
  FIX 1 — Artifact data (display name + description) is now loaded from the
           waypoints .txt file at startup instead of being hardcoded. The UI
           publishes waypoint keys (e.g. "artifact_1") to /artifact_goto so
           navigation_node can look them up directly in its own WAYPOINTS dict.
           Facilities publish special tokens "nearest_toilet" / "nearest_fire_exit".
           The old UI_TO_WAYPOINT mapping in navigation_node is removed.

  FIX 2 — navigator.cancelTask() is called directly from the cancel callback
           in navigation_node (see navigation.py). The Stop button in the UI
           publishes /navigation/cancel which triggers that immediate cancel.
           No change needed here beyond confirming the publish is correct —
           it was already publishing to /navigation/cancel on Stop. ✓
"""

import os
import threading
import subprocess
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from gtts import gTTS

try:
    import ollama
except ImportError:
    ollama = None


# =============================================================================
# Waypoints file loader — reads display names and descriptions for artifact_*
# =============================================================================

DEFAULT_WAYPOINTS_FILE = os.path.join(
    os.path.expanduser('~'),
    'git/RS2Team8/r2_dTour/0_maps/simulation_waypoints.txt',
)

# Fallback artifact data used when the waypoints file cannot be read or has
# no display_name/description fields for a waypoint.
FALLBACK_ARTIFACTS = {
    'artifact_1': {
        'display_name': 'Wooden Toy Truck',
        'keywords': ['truck', 'toy truck', 'wooden truck', 'vehicle', 'car'],
        'response': 'This is a wooden toy truck. It shows simple toy design, natural materials, and playful construction.',
        'info': 'This wooden toy truck represents playful object design. It combines wood, painted panels, wheels, and a small cargo tray to show how everyday toys can be both functional and decorative.',
    },
    'artifact_2': {
        'display_name': 'Pokemon Card',
        'keywords': ['pokemon', 'card', 'japanese card', 'dedenne', 'trading card'],
        'response': 'This is a Japanese Pokémon card. It shows character design, game culture, and collectible artwork.',
        'info': 'This Japanese Pokémon card features Dedenne. It represents modern collectible culture, combining illustration, game mechanics, character branding, and printed design.',
    },
    'artifact_3': {
        'display_name': 'Plush Mouse',
        'keywords': ['mouse', 'plush', 'soft toy', 'rat', 'stuffed toy'],
        'response': 'This is a grey plush mouse. It shows soft material design and character-based toy making.',
        'info': 'This grey plush mouse uses soft fabric, stitched details, and simple facial features to create a friendly character. It represents comfort objects and textile-based toy design.',
    },
    'artifact_4': {
        'display_name': 'Water Bottle',
        'keywords': ['bottle', 'water bottle', 'larq', 'pink bottle', 'drink bottle'],
        'response': 'This is a pink LARQ water bottle. It shows modern product design focused on daily use and sustainability.',
        'info': 'This pink LARQ water bottle represents contemporary product design. Its simple cylindrical form, reusable function, and minimal branding connect to sustainability and everyday lifestyle objects.',
    },
}

# Artefact button colours — keyed by waypoint key, used by the UI layout.
ARTIFACT_COLORS = {
    'artifact_1': '#7b1fa2',
    'artifact_2': '#1565c0',
    'artifact_3': '#ef6c00',
    'artifact_4': '#2e7d32',
}


def load_artifact_waypoints(filepath: str) -> dict:
    """
    Parse the waypoints .txt file and return a dict of artifact entries:
        {
            'artifact_1': {
                'display_name': 'Wooden Toy Truck',
                'info': 'This wooden toy truck ...',
                'keywords': [...],   # from FALLBACK if not in file
                'response': '...',   # from FALLBACK if not in file
            },
            ...
        }

    Only lines whose name starts with 'artifact_' are returned.
    Lines with fewer than 6 fields (name x y yaw display_name description) fall
    back to FALLBACK_ARTIFACTS for display_name/info; coordinate-only lines are
    still accepted but will have no display name or description.

    Underscores in display_name and description fields are replaced with spaces.
    """
    artifacts = {}

    if not os.path.isfile(filepath):
        print(f'[ui_node] WARNING: waypoints file not found: {filepath}. Using fallback data.')
        return {k: dict(v) for k, v in FALLBACK_ARTIFACTS.items()}

    try:
        with open(filepath, 'r') as fh:
            for raw in fh:
                line = raw.split('#')[0].strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 4:
                    continue
                name = parts[0].lower()
                if not name.startswith('artifact_'):
                    continue

                # Start with fallback data so keywords/response are always present
                entry = dict(FALLBACK_ARTIFACTS.get(name, {
                    'display_name': name.replace('_', ' ').title(),
                    'keywords': [],
                    'response': f'This is {name.replace("_", " ")}.',
                    'info': '',
                }))

                # Override display_name and info from file if present
                if len(parts) >= 5:
                    entry['display_name'] = parts[4].replace('_', ' ')
                if len(parts) >= 6:
                    entry['info'] = parts[5].replace('_', ' ')

                artifacts[name] = entry

    except Exception as exc:
        print(f'[ui_node] WARNING: error reading waypoints file: {exc}. Using fallback data.')
        return {k: dict(v) for k, v in FALLBACK_ARTIFACTS.items()}

    if not artifacts:
        print('[ui_node] WARNING: no artifact_* entries found in waypoints file. Using fallback data.')
        return {k: dict(v) for k, v in FALLBACK_ARTIFACTS.items()}

    return artifacts


# =============================================================================
# GreetingNode — ROS2 node
# =============================================================================

class GreetingNode(Node):
    def __init__(self, app):
        super().__init__('greeting_screen_node')
        self.app = app
        self.is_busy = False
        self.stop_requested = False
        self.audio_process = None
        self.plan_sequence = []
        self.plan_index = 0
        self.plan_active = False
        self.plan_arriving = False

        self.ollama_model = 'llama3.2:3b'

        # ── Load artifacts from waypoints file ───────────────────────────────
        # ROS2 parameter allows overriding the path at launch, matching the
        # same parameter used by navigation_node.
        self.declare_parameter('waypoints_file', DEFAULT_WAYPOINTS_FILE)
        waypoints_file = (
            self.get_parameter('waypoints_file')
            .get_parameter_value()
            .string_value
        )

        # self.artifacts  : { waypoint_key -> {display_name, keywords, response, info} }
        # self.display_to_key : { display_name_lower -> waypoint_key }
        self.artifacts = load_artifact_waypoints(waypoints_file)
        self.display_to_key = {
            v['display_name'].lower(): k
            for k, v in self.artifacts.items()
        }

        self.get_logger().info(
            f'Loaded {len(self.artifacts)} artifact(s) from: {waypoints_file}'
        )
        for key, data in self.artifacts.items():
            self.get_logger().info(f'  {key} -> "{data["display_name"]}"')

        # ── Aliases ──────────────────────────────────────────────────────────
        # Build artifact_aliases from loaded data
        self.artifact_aliases = {
            v['display_name'].lower(): v.get('keywords', [])
            for v in self.artifacts.values()
        }

        self.facility_aliases = {
            'fire exit': ['fire exit', 'emergency exit', 'exit'],
            'toilet': ['toilet', 'bathroom', 'restroom', 'washroom'],
        }

        # ── ROS2 parameters for speech messages ──────────────────────────────
        self.declare_parameter('greeting_message',    'Hello. Welcome to the gallery tour robot.')
        self.declare_parameter('instruction_message_1', 'To begin, please select an option on the screen.')
        self.declare_parameter('instruction_message_2', 'You can start a tour, ask for an explanation, type an artefact prompt, or use the selection buttons.')
        self.declare_parameter('instruction_message_3', 'You can also press stop at any time to interrupt the current response.')
        self.declare_parameter('ready_message',       'Ready for user input.')

        self.greeting_message     = self.get_parameter('greeting_message').get_parameter_value().string_value
        self.instruction_message_1 = self.get_parameter('instruction_message_1').get_parameter_value().string_value
        self.instruction_message_2 = self.get_parameter('instruction_message_2').get_parameter_value().string_value
        self.instruction_message_3 = self.get_parameter('instruction_message_3').get_parameter_value().string_value
        self.ready_message        = self.get_parameter('ready_message').get_parameter_value().string_value

        self.facilities = {
            'fire exit': 'The nearest emergency fire exit is located toward the marked exit route. Please follow the emergency exit signage.',
            'toilet':    'The toilet is located in the visitor facilities area. Please follow the amenities signage.',
        }

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Bool,   '/tour_start',       self.start_callback,     10)
        self.create_subscription(Bool,   '/tour_stop',        self.stop_callback,      10)
        self.create_subscription(String, '/artifact_prompt',  self.prompt_callback,    10)
        self.create_subscription(String, '/artifact_goto',    self.goto_callback,      10)
        self.create_subscription(String, '/artifact_info',    self.info_callback,      10)
        self.create_subscription(String, '/artifact_selection', self.selection_callback, 10)
        self.create_subscription(String, '/navigation_status', self.nav_status_callback, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.navigation_publisher        = self.create_publisher(Bool,   '/start_navigation',    10)
        self.selected_artifact_publisher = self.create_publisher(String, '/selected_artifact',   10)
        self.goto_publisher              = self.create_publisher(String, '/artifact_goto',       10)
        self.artifact_info_publisher     = self.create_publisher(String, '/artifact_info_request', 10)

        self.get_logger().info('Greeting screen node ready. Waiting for /tour_start')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _resolve_display_to_key(self, display_name: str) -> str | None:
        """Return the waypoint key for a given display name (case-insensitive)."""
        return self.display_to_key.get(display_name.lower())

    def _key_to_display(self, key: str) -> str:
        """Return the human-readable display name for a waypoint key."""
        entry = self.artifacts.get(key)
        return entry['display_name'] if entry else key.replace('_', ' ').title()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def start_callback(self, msg):
        if not msg.data:
            return
        if self.is_busy:
            self.get_logger().info('Already running greeting sequence.')
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.cancel_idle_timer()
        self.app.set_status('Processing')
        self.app.set_text('Processing request...')
        self.app.set_buttons_enabled(False)
        threading.Thread(target=self.run_sequence, daemon=True).start()

    def stop_callback(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Stop command received.')
        self.stop_requested = True
        if self.audio_process is not None:
            try:
                self.audio_process.terminate()
            except Exception:
                pass
        if self.plan_active:
            self.plan_active = False
            self.app.end_plan()
        self.app.cancel_idle_timer()
        self.app.set_status('Listening')
        self.app.set_text('Stopping... Ready for user input.')
        self.app.set_buttons_enabled(True)
        self.is_busy = False

    def prompt_callback(self, msg):
        prompt = msg.data.strip()
        if not prompt:
            return
        if self.is_busy:
            self.get_logger().info('Busy speaking. Ignoring artefact prompt.')
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.cancel_idle_timer()
        self.app.set_status('Processing')
        self.app.set_text('Analysing user prompt...')
        self.app.set_buttons_enabled(False)
        threading.Thread(target=self.choose_artifact_from_prompt, args=(prompt,), daemon=True).start()

    def goto_callback(self, msg):
        selection = msg.data.strip().lower()
        if not selection:
            return
        if self.is_busy:
            self.get_logger().info('Busy. Ignoring Go To request.')
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.cancel_idle_timer()
        self.app.set_status('Processing')
        # selection here is a waypoint key — get display name for UI
        display = self._key_to_display(selection) if selection in self.artifacts else selection.title()
        self.app.set_text(f'Navigating to {display}...')
        self.app.set_buttons_enabled(False)
        self.app.set_destination(display)
        threading.Thread(target=self.handle_goto, args=(selection,), daemon=True).start()

    def info_callback(self, msg):
        selection = msg.data.strip().lower()
        if not selection:
            return
        if self.is_busy:
            self.get_logger().info('Busy. Ignoring Info request.')
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.cancel_idle_timer()
        self.app.set_status('Processing')
        display = self._key_to_display(selection) if selection in self.artifacts else selection.title()
        self.app.set_text(f'Loading information for {display}...')
        self.app.set_buttons_enabled(False)
        threading.Thread(target=self.handle_info, args=(selection,), daemon=True).start()

    def selection_callback(self, msg):
        selection = msg.data.strip().lower()
        if not selection:
            return
        if self.is_busy:
            self.get_logger().info('Busy speaking. Ignoring selection.')
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.cancel_idle_timer()
        self.app.set_status('Processing')
        self.app.set_text('Processing selection...')
        self.app.set_buttons_enabled(False)
        threading.Thread(target=self.handle_selection, args=(selection,), daemon=True).start()

    def nav_status_callback(self, msg):
        status = msg.data.strip()
        self.get_logger().info(f'Navigation status received: {status}')
        self.app.set_nav_status(status)
        status_map = {
            'IDLE':       'Listening',
            'ROTATING':   'Navigating',
            'NAVIGATING': 'Navigating',
            'OBSTACLE':   'Navigating',
            'REROUTING':  'Rerouting',
            'STUCK':      'Error',
            'FAILED':     'Error',
            'REACHED':    'Listening',
        }
        if status in status_map:
            self.app.set_status(status_map[status])
        if self.plan_active and not self.plan_arriving:
            if status == 'REACHED':
                self.plan_arriving = True
                threading.Thread(target=self.handle_plan_arrival, daemon=True).start()
            elif status in ('FAILED', 'STUCK'):
                self.plan_active = False
                self.is_busy = False
                self.app.end_plan()
                self.app.set_buttons_enabled(True)
                self.app.set_text('Navigation failed. Tour plan cancelled.')
        elif not self.plan_active:
            if status in ('REACHED', 'FAILED', 'STUCK', 'IDLE'):
                self.is_busy = False
                self.app.set_buttons_enabled(True)

    # ── Plan control ──────────────────────────────────────────────────────────

    def start_plan(self, sequence):
        """sequence is a list of waypoint keys, e.g. ['artifact_1', 'artifact_3']"""
        if not sequence:
            return
        self.plan_sequence = list(sequence)
        self.plan_index = 0
        self.plan_active = True
        self.plan_arriving = False
        self.is_busy = True
        self.stop_requested = False
        self._navigate_to_plan_stop()

    def next_stop(self):
        if not self.plan_active:
            return
        self.plan_index += 1
        if self.plan_index >= len(self.plan_sequence):
            return
        self.is_busy = True
        self.stop_requested = False
        self.app.set_buttons_enabled(False)
        self._navigate_to_plan_stop()

    def _navigate_to_plan_stop(self):
        """
        Publish the waypoint key for the current plan stop directly to
        /artifact_goto. navigation_node looks it up in its WAYPOINTS dict.
        """
        waypoint_key = self.plan_sequence[self.plan_index]
        display = self._key_to_display(waypoint_key)
        stop_num = self.plan_index + 1
        total = len(self.plan_sequence)
        self.app.set_status('Navigating')
        self.app.set_text(f'Navigating to stop {stop_num} of {total}: {display}...')
        self.app.set_destination(display)
        self.app.set_nav_status(f'Plan stop {stop_num}/{total}: {display}')
        goto_msg = String()
        goto_msg.data = waypoint_key   # FIX 1: publish key, not display name
        self.goto_publisher.publish(goto_msg)

    def handle_plan_arrival(self):
        try:
            waypoint_key = self.plan_sequence[self.plan_index]
            display = self._key_to_display(waypoint_key)
            total = len(self.plan_sequence)
            stop_num = self.plan_index + 1

            if waypoint_key in self.artifacts:
                info_text = self.artifacts[waypoint_key]['info']
                self.app.set_info_panel(display, info_text)
                speech = f'You have arrived at the {display} exhibit. {info_text}'
            else:
                speech = f'You have arrived at {display}.'

            self.app.set_status('Speaking')
            self.app.set_text(f'Stop {stop_num} of {total}: {display}')
            self.speak_text(speech)

            if self.stop_requested:
                self.plan_active = False
                self.is_busy = False
                self.app.end_plan()
                self.return_to_listening('Tour plan stopped.')
                return

            if self.plan_index >= total - 1:
                completion = 'That concludes your planned tour. Thank you for visiting.'
                self.app.set_status('Speaking')
                self.app.set_text(completion)
                self.speak_text(completion)
                if not self.stop_requested:
                    self.plan_active = False
                    self.is_busy = False
                    self.app.end_plan()
                    self.return_to_listening('Tour complete. Ready for user input.')
            else:
                self.is_busy = False
                self.app.set_status('Listening')
                self.app.set_text(
                    f'Stop {stop_num} of {total} complete. Press Next Stop when ready to continue.'
                )
                self.app.enable_next_stop_button()

        except Exception as e:
            self.get_logger().error(f'Plan arrival error: {e}')
            self.plan_active = False
            self.is_busy = False
            self.app.end_plan()
            self.app.set_status('Error')
            self.app.set_text(f'Plan error: {e}')
        finally:
            self.plan_arriving = False

    # ── Sequence / speech handlers ────────────────────────────────────────────

    def run_sequence(self):
        try:
            self.play_beep()
            if self.stop_requested:
                return
            for message in [
                self.greeting_message,
                self.instruction_message_1,
                self.instruction_message_2,
                self.instruction_message_3,
            ]:
                self.app.set_status('Speaking')
                self.app.set_text(message)
                self.speak_text(message)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return

            self.return_to_listening(self.ready_message)
            nav_msg = Bool()
            nav_msg.data = True
            self.navigation_publisher.publish(nav_msg)
            self.get_logger().info('Published /start_navigation trigger.')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.app.set_status('Error')
            self.app.set_text(f'Error: {e}')
        finally:
            self.audio_process = None
            self.is_busy = False
            self.stop_requested = False
            self.app.set_buttons_enabled(True)

    def handle_goto(self, selection):
        """
        selection is a waypoint key (e.g. 'artifact_1'), a facility name
        ('fire exit', 'toilet'), or a pre-resolved special token
        ('nearest_fire_exit', 'nearest_toilet') sent by publish_goto.

        All three forms are handled so navigation AND the spoken confirmation
        both work correctly.
        """
        # Normalise special tokens back to their facility name for display/speech
        SPECIAL_TO_FACILITY = {
            'nearest_fire_exit': 'fire exit',
            'nearest_toilet':    'toilet',
        }
        if selection in SPECIAL_TO_FACILITY:
            selection = SPECIAL_TO_FACILITY[selection]

        try:
            if selection in self.artifacts:
                display = self._key_to_display(selection)
                speech = f'Navigating to the {display} exhibit. Please follow the robot.'
                self.app.set_status('Speaking')
                self.app.set_text(speech)
                self.speak_text(speech)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return
                # Publish waypoint key — navigation_node looks it up directly
                if self.goto_publisher is not None:
                    goto_msg = String()
                    goto_msg.data = selection  # e.g. "artifact_1"
                    self.goto_publisher.publish(goto_msg)
                    self.get_logger().info(f'Published /artifact_goto: {selection}')
                selected_msg = String()
                selected_msg.data = selection
                self.selected_artifact_publisher.publish(selected_msg)
                self.app.set_status('Navigating')
                self.app.set_nav_status(f'Navigating to {display}...')
                self.return_to_listening(f'Navigating to {display}. Ready for input.')
                return

            if selection in self.facilities:
                display = selection.title()
                speech = f'Guiding you to the {display}. Please follow the robot.'
                self.app.set_status('Speaking')
                self.app.set_text(speech)
                self.speak_text(speech)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return
                # Publish special tokens for facility navigation
                if self.goto_publisher is not None:
                    goto_msg = String()
                    if selection == 'toilet':
                        goto_msg.data = 'nearest_toilet'
                    else:
                        goto_msg.data = 'nearest_fire_exit'
                    self.goto_publisher.publish(goto_msg)
                    self.get_logger().info(f'Published /artifact_goto: {goto_msg.data}')
                self.app.set_status('Navigating')
                self.app.set_nav_status(f'Navigating to {display}...')
                self.return_to_listening(f'Navigating to {display}. Ready for input.')
                return

            error_message = 'Sorry, that destination is not available.'
            self.app.set_status('Speaking')
            self.app.set_text(error_message)
            self.speak_text(error_message)
            self.return_to_listening('Invalid destination. Ready for user input.')

        except Exception as e:
            self.get_logger().error(f'Go To handling error: {e}')
            self.app.set_status('Error')
            self.app.set_text(f'Error: {e}')
        finally:
            self.audio_process = None
            self.is_busy = False
            self.stop_requested = False
            self.app.set_buttons_enabled(True)

    def handle_info(self, selection):
        """selection is a waypoint key (e.g. 'artifact_1')."""
        try:
            if selection in self.artifacts:
                display = self._key_to_display(selection)
                info_text = self.artifacts[selection]['info']
                self.app.set_info_panel(display, info_text)
                self.app.set_status('Speaking')
                self.app.set_text(info_text)
                self.speak_text(info_text)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return
                self.return_to_listening(f'Information provided for {display}. Ready for user input.')
                return

            error_message = 'Sorry, no information is available for that selection.'
            self.app.set_status('Speaking')
            self.app.set_text(error_message)
            self.speak_text(error_message)
            self.return_to_listening('Ready for user input.')

        except Exception as e:
            self.get_logger().error(f'Info handling error: {e}')
            self.app.set_status('Error')
            self.app.set_text(f'Error: {e}')
        finally:
            self.audio_process = None
            self.is_busy = False
            self.stop_requested = False
            self.app.set_buttons_enabled(True)

    def infer_known_selection(self, text):
        text_lower = text.lower()
        for artifact_key, data in self.artifacts.items():
            display_lower = data['display_name'].lower()
            if display_lower in text_lower:
                return artifact_key, 'artifact'
            for alias in data.get('keywords', []):
                if alias in text_lower:
                    return artifact_key, 'artifact'
        for facility_name, aliases in self.facility_aliases.items():
            for alias in aliases:
                if alias in text_lower:
                    return facility_name, 'facility'
        return None, None

    def get_ai_response(self, user_prompt, mode='general', selected_item=None):
        if ollama is None:
            return 'Ollama is not installed. Please install it to use AI responses.'
        if self.stop_requested:
            return 'Stopping...'

        prompt_lower = user_prompt.lower().strip()

        if 'fire exit' in prompt_lower or 'emergency exit' in prompt_lower:
            return (
                'In an emergency, please follow the emergency exit signs '
                'and staff instructions to reach the nearest fire exit safely.'
            )
        if any(w in prompt_lower for w in ('toilet', 'bathroom', 'restroom', 'washroom')):
            return (
                'The toilets are located in the visitor facilities area. '
                'Please follow the amenities signs.'
            )

        artifact_list = ', '.join(
            data['display_name'] for data in self.artifacts.values()
        )

        try:
            system_prompt = (
                'You are a gallery tour guide robot. '
                'Give short spoken responses using 1 to 3 short sentences only. '
                'Do not use bullet points. Do not say you are an AI. '
                f'You can only talk about these four artefacts: {artifact_list}. '
                'You can describe each artefact\'s characteristics, colour, shape, and history. '
                'If asked about an emergency or fire exit, tell the user to follow the signs and offer to guide them. '
                'If asked about washrooms or toilets, guide the user to the bathroom. '
                'If the visitor asks about anything else, politely say that only these four artefacts are available in this gallery. '
                'After providing information, ask if they would like to be guided to that artefact — they can press the "Go To" button. '
                'If you receive instruction in a different language, reply in the same language.'
            )
            response = ollama.chat(
                model=self.ollama_model,
                messages=[
                    {'role': 'system', 'content': system_prompt},
                    {'role': 'user', 'content': f'The visitor asked: "{user_prompt}". Reply briefly.'},
                ],
                options={'temperature': 0.6},
            )
            text = response.get('message', {}).get('content', '').strip()
            return text if text else 'No response generated.'
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return 'AI error. Please try again.'

    def choose_artifact_from_prompt(self, prompt):
        try:
            matched_item, matched_type = self.infer_known_selection(prompt)

            if matched_item and matched_type == 'artifact':
                display = self._key_to_display(matched_item)
                selected_msg = String()
                selected_msg.data = matched_item
                self.selected_artifact_publisher.publish(selected_msg)
                self.app.set_info_panel(display, self.artifacts[matched_item]['info'])
                self.app.set_destination(display)

            response = self.get_ai_response(prompt, mode='general', selected_item=matched_item)

            self.app.set_status('Speaking')
            self.app.set_text(response)
            self.speak_text(response)

            if self.stop_requested:
                self.return_to_listening('Stopping... Ready for user input.')
                return

            self.return_to_listening('AI response completed. Ready for user input.')

        except Exception as e:
            self.get_logger().error(f'Artifact selection error: {e}')
            self.app.set_status('Error')
            self.app.set_text(f'Artifact selection error: {e}')
        finally:
            self.audio_process = None
            self.is_busy = False
            self.stop_requested = False
            self.app.set_buttons_enabled(True)

    def handle_selection(self, selection):
        """selection is a waypoint key or facility name."""
        try:
            if selection in self.artifacts:
                display = self._key_to_display(selection)
                response = self.artifacts[selection]['response']
                selected_msg = String()
                selected_msg.data = selection
                self.selected_artifact_publisher.publish(selected_msg)
                full_response = f'You selected {display}. {response}'
                self.app.set_status('Speaking')
                self.app.set_text(full_response)
                self.speak_text(full_response)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return
                self.return_to_listening(f'Selected artefact: {display}. Ready for user input.')
                return

            if selection in self.facilities:
                response = self.facilities[selection]
                self.app.set_status('Speaking')
                self.app.set_text(response)
                self.speak_text(response)
                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return
                self.return_to_listening(f'{selection.title()} information provided. Ready for user input.')
                return

            error_message = 'Sorry, that selection is not available.'
            self.app.set_status('Speaking')
            self.app.set_text(error_message)
            self.speak_text(error_message)
            self.return_to_listening('Invalid selection. Ready for user input.')

        except Exception as e:
            self.get_logger().error(f'Selection handling error: {e}')
            self.app.set_status('Error')
            self.app.set_text(f'Selection handling error: {e}')
        finally:
            self.audio_process = None
            self.is_busy = False
            self.stop_requested = False
            self.app.set_buttons_enabled(True)

    def return_to_listening(self, message):
        self.app.set_status('Listening')
        self.app.set_text(message)
        self.app.start_idle_timer()

    def play_beep(self):
        try:
            subprocess.run(['bash', '-c', 'printf "\\a"'], check=True)
        except Exception:
            self.get_logger().warn('Could not play beep.')

    def speak_text(self, text):
        if self.stop_requested:
            return
        try:
            filename = 'speech.mp3'
            tts = gTTS(text=text, lang='en')
            tts.save(filename)
            self.audio_process = subprocess.Popen(
                ['mpg123', filename],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.audio_process.wait()
            self.audio_process = None
            if os.path.exists(filename):
                os.remove(filename)
        except Exception as e:
            print(f'Speech error: {e}')


# =============================================================================
# GreetingApp — tkinter UI
# =============================================================================

class GreetingApp:
    """
    Tkinter UI.  All waypoint-key <-> display-name translation happens in
    GreetingNode.  The GreetingApp only knows about display names for labels
    and uses publish_goto(display_name) which GreetingNode converts to a key.
    """

    def _f(self, n):
        """Scale font size to screen."""
        return max(9, int(n * self._sf))

    def _p(self, n):
        """Scale padding to screen."""
        return max(2, int(n * self._sf))

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Tour Guide Robot Interface')
        self.root.configure(bg='#f4f6f8')
        self.root.resizable(True, True)

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self._sf = min(sw / 1920, sh / 1080, 1.0)

        try:
            self.root.attributes('-zoomed', True)
        except tk.TclError:
            self.root.geometry(f'{sw}x{sh}')

        self.idle_after_id = None
        self.all_buttons = []

        # ── Header ──────────────────────────────────────────────────────────
        self.top_frame = tk.Frame(self.root, bg='#0b57d0', height=self._p(70))
        self.top_frame.pack(fill='x')
        tk.Label(
            self.top_frame,
            text='Tour Guide Robot Interface',
            font=('Arial', self._f(24), 'bold'),
            fg='white', bg='#0b57d0',
        ).pack(pady=self._p(15))

        # ── Status row ──────────────────────────────────────────────────────
        self.status_frame = tk.Frame(self.root, bg='#f4f6f8')
        self.status_frame.pack(pady=(self._p(14), self._p(4)), fill='x', padx=self._p(40))

        led_sz = self._p(30)
        _m = self._p(5)
        self.led_canvas = tk.Canvas(
            self.status_frame, width=led_sz, height=led_sz,
            bg='#f4f6f8', highlightthickness=0,
        )
        self.led_canvas.pack(side='left', padx=self._p(10))
        self.led = self.led_canvas.create_oval(_m, _m, led_sz - _m, led_sz - _m, fill='gray')

        self.status_var = tk.StringVar(value='Status: Waiting')
        tk.Label(
            self.status_frame, textvariable=self.status_var,
            font=('Arial', self._f(18), 'bold'), fg='#0b57d0', bg='#f4f6f8',
        ).pack(side='left')

        self.destination_var = tk.StringVar(value='Current destination: None')
        tk.Label(
            self.status_frame, textvariable=self.destination_var,
            font=('Arial', self._f(14)), fg='#555555', bg='#f4f6f8',
        ).pack(side='right', padx=self._p(20))

        self.nav_status_var = tk.StringVar(value='Navigation: Idle')
        tk.Label(
            self.root, textvariable=self.nav_status_var,
            font=('Arial', self._f(13), 'italic'), fg='#1a6e38', bg='#f4f6f8',
        ).pack(anchor='e', padx=self._p(44))

        # ── Main speech display ─────────────────────────────────────────────
        self.text_var = tk.StringVar(value='Waiting for user to press screen...')
        self.text_label = tk.Label(
            self.root, textvariable=self.text_var,
            wraplength=sw - self._p(140), justify='center',
            font=('Arial', self._f(20)), bg='white', fg='black',
            padx=self._p(30), pady=self._p(22), relief='ridge', bd=2,
        )
        self.text_label.pack(padx=self._p(40), pady=(self._p(8), self._p(6)), fill='x')

        # ── Tabbed content area ─────────────────────────────────────────────
        style = ttk.Style()
        style.configure(
            'TNotebook.Tab',
            font=('Arial', self._f(14), 'bold'),
            padding=[self._p(24), self._p(8)],
        )
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=self._p(30), pady=(self._p(4), self._p(2)))

        self.tab_artifacts  = tk.Frame(self.notebook, bg='#f4f6f8')
        self.tab_planner    = tk.Frame(self.notebook, bg='#f4f6f8')
        self.tab_facilities = tk.Frame(self.notebook, bg='#f4f6f8')

        self.notebook.add(self.tab_artifacts,  text='  Artefacts  ')
        self.notebook.add(self.tab_planner,    text='  Tour Planner  ')
        self.notebook.add(self.tab_facilities, text='  Facilities  ')

        # ── Tab 1 : Artefacts ───────────────────────────────────────────────
        self.info_frame = tk.LabelFrame(
            self.tab_artifacts, text='Artefact Information',
            font=('Arial', self._f(13), 'bold'), bg='#eaf4fb',
            padx=self._p(14), pady=self._p(8),
        )
        self.info_frame.pack(fill='x', padx=self._p(16), pady=(self._p(10), self._p(6)))

        self.info_title_var = tk.StringVar(value='')
        tk.Label(
            self.info_frame, textvariable=self.info_title_var,
            font=('Arial', self._f(14), 'bold'), bg='#eaf4fb', fg='#0b57d0',
        ).pack(anchor='w')

        self.info_text_var = tk.StringVar(value='Select an artefact and press Info to see details here.')
        self.info_text_label = tk.Label(
            self.info_frame, textvariable=self.info_text_var,
            wraplength=sw - self._p(160), justify='left',
            font=('Arial', self._f(13)), bg='#eaf4fb', fg='#222222',
        )
        self.info_text_label.pack(anchor='w')

        # Artifact rows — display names from FALLBACK; will match what the
        # node loaded. If node loaded different names from the file the buttons
        # still work because publish_goto converts via display_to_key.
        for name, color in [
            ('Wooden Toy Truck', ARTIFACT_COLORS.get('artifact_1', '#7b1fa2')),
            ('Pokemon Card',     ARTIFACT_COLORS.get('artifact_2', '#1565c0')),
            ('Plush Mouse',      ARTIFACT_COLORS.get('artifact_3', '#ef6c00')),
            ('Water Bottle',     ARTIFACT_COLORS.get('artifact_4', '#2e7d32')),
        ]:
            self.create_artifact_row(self.tab_artifacts, name, color)

        self.prompt_frame = tk.Frame(self.tab_artifacts, bg='#f4f6f8')
        self.prompt_frame.pack(pady=(self._p(8), self._p(12)))

        tk.Label(
            self.prompt_frame, text='Artefact prompt:',
            font=('Arial', self._f(14), 'bold'), bg='#f4f6f8',
        ).pack(side='left', padx=self._p(8))

        self.prompt_entry = tk.Entry(
            self.prompt_frame, font=('Arial', self._f(14)),
            width=max(20, int(45 * self._sf)),
        )
        self.prompt_entry.pack(side='left', padx=self._p(8))

        self.prompt_button = tk.Button(
            self.prompt_frame, text='Use Prompt',
            font=('Arial', self._f(14), 'bold'),
            bg='#188038', fg='white',
            activebackground='#146c2e', activeforeground='white',
            padx=self._p(16), pady=self._p(6),
            command=self.publish_prompt,
        )
        self.prompt_button.pack(side='left', padx=self._p(8))
        self.all_buttons.append(self.prompt_button)

        # ── Tab 2 : Tour Planner ────────────────────────────────────────────
        self.plan_sequence = []    # list of waypoint keys
        self.plan_active = False

        self.plan_sequence_var = tk.StringVar(value='Plan: No stops selected.')
        tk.Label(
            self.tab_planner, textvariable=self.plan_sequence_var,
            font=('Arial', self._f(14)), bg='#f4f6f8', fg='#444444', anchor='w',
        ).pack(fill='x', padx=self._p(16), pady=(self._p(14), self._p(8)))

        self.planner_btn_frame = tk.Frame(self.tab_planner, bg='#f4f6f8')
        self.planner_btn_frame.pack(fill='x', padx=self._p(16), pady=(0, self._p(8)))

        self.planner_artifact_buttons = {}
        for name, color in [
            ('Wooden Toy Truck', ARTIFACT_COLORS.get('artifact_1', '#7b1fa2')),
            ('Pokemon Card',     ARTIFACT_COLORS.get('artifact_2', '#1565c0')),
            ('Plush Mouse',      ARTIFACT_COLORS.get('artifact_3', '#ef6c00')),
            ('Water Bottle',     ARTIFACT_COLORS.get('artifact_4', '#2e7d32')),
        ]:
            btn = tk.Button(
                self.planner_btn_frame, text=f'+ {name}',
                font=('Arial', self._f(13), 'bold'),
                bg=color, fg='white', activeforeground='white',
                padx=self._p(14), pady=self._p(10),
                command=lambda n=name: self.add_to_plan(n),
            )
            btn.pack(side='left', padx=self._p(6))
            self.planner_artifact_buttons[name] = btn
            self.all_buttons.append(btn)

        self.planner_ctrl_frame = tk.Frame(self.tab_planner, bg='#f4f6f8')
        self.planner_ctrl_frame.pack(fill='x', padx=self._p(16))

        self.plan_undo_button = tk.Button(
            self.planner_ctrl_frame, text='Undo Last',
            font=('Arial', self._f(13), 'bold'),
            bg='#78909c', fg='white', activeforeground='white',
            padx=self._p(14), pady=self._p(10),
            command=self.undo_plan_step,
        )
        self.plan_undo_button.pack(side='left', padx=self._p(6))
        self.all_buttons.append(self.plan_undo_button)

        self.plan_clear_button = tk.Button(
            self.planner_ctrl_frame, text='Clear',
            font=('Arial', self._f(13), 'bold'),
            bg='#78909c', fg='white', activeforeground='white',
            padx=self._p(14), pady=self._p(10),
            command=self.clear_plan,
        )
        self.plan_clear_button.pack(side='left', padx=self._p(6))
        self.all_buttons.append(self.plan_clear_button)

        # Start Plan / Next Stop — managed separately, NOT in all_buttons
        self.plan_start_button = tk.Button(
            self.planner_ctrl_frame, text='Start Plan',
            font=('Arial', self._f(13), 'bold'),
            bg='#0b57d0', fg='white', activeforeground='white',
            padx=self._p(18), pady=self._p(10),
            state='disabled',
            command=self.handle_plan_button,
        )
        self.plan_start_button.pack(side='left', padx=self._p(14))

        # ── Tab 3 : Facilities ──────────────────────────────────────────────
        tk.Label(
            self.tab_facilities,
            text='Select a facility to navigate to:',
            font=('Arial', self._f(15)), bg='#f4f6f8', fg='#444444',
        ).pack(pady=(self._p(22), self._p(14)))

        self.location_button_frame = tk.Frame(self.tab_facilities, bg='#f4f6f8')
        self.location_button_frame.pack()

        self.create_goto_button(self.location_button_frame, 'Fire Exit', '#c62828')
        self.create_goto_button(self.location_button_frame, 'Toilet',    '#00838f')

        # ── Control buttons — always visible below tabs ──────────────────────
        self.button_frame = tk.Frame(self.root, bg='#f4f6f8')
        self.button_frame.pack(pady=self._p(10))

        self.start_button = tk.Button(
            self.button_frame, text='Start Greeting',
            font=('Arial', self._f(16), 'bold'),
            bg='#0b57d0', fg='white',
            activebackground='#0842a0', activeforeground='white',
            padx=self._p(20), pady=self._p(10),
            command=self.publish_start,
        )
        self.start_button.pack(side='left', padx=self._p(10))
        self.all_buttons.append(self.start_button)

        self.stop_button = tk.Button(
            self.button_frame, text='Stop',
            font=('Arial', self._f(16), 'bold'),
            bg='#d93025', fg='white',
            activebackground='#b3261e', activeforeground='white',
            padx=self._p(20), pady=self._p(10),
            command=self.publish_stop,
        )
        self.stop_button.pack(side='left', padx=self._p(10))
        # Stop button always stays enabled — do NOT add to all_buttons

        # ── ROS publishers (set after attach_node) ──────────────────────────
        self.node = None
        self.start_publisher      = None
        self.stop_publisher       = None
        self.nav_cancel_publisher = None
        self.prompt_publisher     = None
        self.goto_publisher       = None
        self.info_publisher       = None

        self.root.bind('<Configure>', self._on_resize)

    # ── Widget factories ─────────────────────────────────────────────────────

    def create_artifact_row(self, parent, label, color):
        """label is the display name (e.g. 'Wooden Toy Truck')."""
        row = tk.Frame(parent, bg='#f4f6f8')
        row.pack(fill='x', pady=self._p(4))

        tk.Label(
            row, text=label,
            font=('Arial', self._f(14), 'bold'),
            width=22, anchor='w', bg='#f4f6f8',
        ).pack(side='left', padx=self._p(10))

        goto_btn = tk.Button(
            row, text='Go To',
            font=('Arial', self._f(13), 'bold'),
            bg=color, fg='white', activeforeground='white',
            padx=self._p(18), pady=self._p(8),
            command=lambda l=label: self.publish_goto(l),
        )
        goto_btn.pack(side='left', padx=self._p(6))
        self.all_buttons.append(goto_btn)

        info_btn = tk.Button(
            row, text='Info',
            font=('Arial', self._f(13), 'bold'),
            bg='#455a64', fg='white', activeforeground='white',
            padx=self._p(18), pady=self._p(8),
            command=lambda l=label: self.publish_info(l),
        )
        info_btn.pack(side='left', padx=self._p(6))
        self.all_buttons.append(info_btn)

    def create_goto_button(self, parent, label, color):
        """label is a facility display name (e.g. 'Fire Exit', 'Toilet')."""
        btn = tk.Button(
            parent, text=f'Go To {label}',
            font=('Arial', self._f(14), 'bold'),
            bg=color, fg='white', activeforeground='white',
            padx=self._p(18), pady=self._p(10),
            command=lambda l=label: self.publish_goto(l),
        )
        btn.pack(side='left', padx=self._p(10), pady=self._p(5))
        self.all_buttons.append(btn)

    # ── Node attachment ───────────────────────────────────────────────────────

    def attach_node(self, node):
        self.node = node
        self.start_publisher      = node.create_publisher(Bool,   '/tour_start',         10)
        self.stop_publisher       = node.create_publisher(Bool,   '/tour_stop',          10)
        self.nav_cancel_publisher = node.create_publisher(String, '/navigation/cancel',  10)
        self.prompt_publisher     = node.create_publisher(String, '/artifact_prompt',    10)
        self.goto_publisher       = node.create_publisher(String, '/artifact_goto',      10)
        self.info_publisher       = node.create_publisher(String, '/artifact_info',      10)

    # ── UI state helpers ──────────────────────────────────────────────────────

    def cancel_idle_timer(self):
        if self.idle_after_id is not None:
            self.root.after_cancel(self.idle_after_id)
            self.idle_after_id = None

    def set_text(self, text):
        self.root.after(0, lambda: self.text_var.set(text))

    def set_status(self, status):
        def update():
            self.status_var.set(f'Status: {status}')
            color_map = {
                'Listening':  'green',
                'Speaking':   'orange',
                'Processing': 'blue',
                'Navigating': 'purple',
                'Rerouting':  '#e65c00',
                'Error':      'red',
            }
            color = color_map.get(status, 'gray')
            if status != 'Listening':
                self.cancel_idle_timer()
            self.led_canvas.itemconfig(self.led, fill=color)
        self.root.after(0, update)

    def set_nav_status(self, status_text):
        self.root.after(0, lambda: self.nav_status_var.set(f'Navigation: {status_text}'))

    def set_buttons_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.root.after(0, lambda: [btn.config(state=state) for btn in self.all_buttons])

    def set_info_panel(self, title, text):
        self.root.after(0, lambda: self.info_title_var.set(title))
        self.root.after(0, lambda: self.info_text_var.set(text))

    def set_destination(self, destination):
        self.root.after(0, lambda: self.destination_var.set(f'Current destination: {destination}'))

    def start_idle_timer(self):
        def reset_to_idle():
            self.status_var.set('Status: Waiting')
            self.text_var.set('Waiting for user to press screen...')
            self.led_canvas.itemconfig(self.led, fill='gray')
            self.idle_after_id = None
        self.cancel_idle_timer()
        self.idle_after_id = self.root.after(10000, reset_to_idle)

    # ── Publish actions ───────────────────────────────────────────────────────

    def publish_start(self):
        if self.start_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_text('Processing request...')
            msg = Bool()
            msg.data = True
            self.start_publisher.publish(msg)

    def publish_stop(self):
        """
        FIX 2: publishes to /navigation/cancel which now calls
        navigator.cancelTask() immediately in navigation_node, so a single
        press interrupts navigation without needing to be spammed.
        """
        self.cancel_idle_timer()
        self.set_status('Listening')
        self.set_text('Stopping...')
        self.set_nav_status('Idle')
        self.set_buttons_enabled(True)
        if self.stop_publisher is not None:
            msg = Bool()
            msg.data = True
            self.stop_publisher.publish(msg)
        if self.nav_cancel_publisher is not None:
            msg = String()
            msg.data = 'cancel'
            self.nav_cancel_publisher.publish(msg)

    def publish_prompt(self):
        if self.prompt_publisher is not None:
            prompt = self.prompt_entry.get().strip()
            if prompt:
                self.cancel_idle_timer()
                self.set_status('Processing')
                self.set_text('Analysing user prompt...')
                msg = String()
                msg.data = prompt
                self.prompt_publisher.publish(msg)

    def publish_goto(self, selection):
        """
        selection is a display name from a button label (e.g. 'Wooden Toy Truck',
        'Fire Exit', 'Toilet').

        For artefacts: convert to waypoint key (e.g. 'artifact_1') via the
        node's display_to_key map before publishing to /artifact_goto, so
        navigation_node receives a key it can look up directly.

        For facilities: convert to the special tokens 'nearest_toilet' or
        'nearest_fire_exit' that navigation_node resolves at runtime.
        """
        if self.goto_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_destination(selection)
            self.set_text(f'Navigating to {selection}...')

            key = selection.lower()
            # Convert display name to waypoint key if the node is attached
            if self.node is not None:
                artifact_key = self.node.display_to_key.get(key)
                if artifact_key:
                    key = artifact_key               # e.g. "artifact_1"
                elif key == 'toilet':
                    key = 'nearest_toilet'
                elif key in ('fire exit',):
                    key = 'nearest_fire_exit'

            msg = String()
            msg.data = key
            self.goto_publisher.publish(msg)

    def publish_info(self, selection):
        """
        selection is a display name (e.g. 'Wooden Toy Truck').
        Convert to waypoint key before publishing to /artifact_info.
        """
        if self.info_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_text(f'Loading information for {selection}...')

            key = selection.lower()
            if self.node is not None:
                artifact_key = self.node.display_to_key.get(key)
                if artifact_key:
                    key = artifact_key

            msg = String()
            msg.data = key
            self.info_publisher.publish(msg)

    # ── Tour planner ──────────────────────────────────────────────────────────

    def add_to_plan(self, name):
        """name is a display name (e.g. 'Wooden Toy Truck')."""
        key = name.lower()
        if key not in self.plan_sequence:
            self.plan_sequence.append(key)
            self.update_plan_display()
            self.update_plan_start_button()

    def undo_plan_step(self):
        if self.plan_sequence:
            self.plan_sequence.pop()
            self.update_plan_display()
            self.update_plan_start_button()

    def clear_plan(self):
        self.plan_sequence.clear()
        self.update_plan_display()
        self.update_plan_start_button()

    def update_plan_display(self):
        if not self.plan_sequence:
            self.plan_sequence_var.set('Plan: No stops selected.')
        else:
            stops = ' → '.join(f'{i + 1}. {s.title()}' for i, s in enumerate(self.plan_sequence))
            self.plan_sequence_var.set(f'Plan: {stops}')

    def update_plan_start_button(self):
        if not self.plan_active:
            state = 'normal' if self.plan_sequence else 'disabled'
            self.plan_start_button.config(text='Start Plan', state=state, bg='#0b57d0')

    def handle_plan_button(self):
        if not self.plan_active:
            if self.node and self.plan_sequence:
                # Resolve display-name keys to waypoint keys before handing off
                resolved = []
                for display_key in self.plan_sequence:
                    # display_key is already lowercased display name from add_to_plan
                    waypoint_key = self.node.display_to_key.get(display_key, display_key)
                    resolved.append(waypoint_key)
                self.plan_active = True
                self.plan_start_button.config(text='Next Stop', state='disabled', bg='#e65c00')
                self.set_buttons_enabled(False)
                self.node.start_plan(resolved)
        else:
            self.plan_start_button.config(state='disabled')
            self.set_buttons_enabled(False)
            self.node.next_stop()

    def enable_next_stop_button(self):
        self.root.after(0, lambda: self.plan_start_button.config(
            text='Next Stop', state='normal', bg='#e65c00',
        ))

    def end_plan(self):
        def reset():
            self.plan_active = False
            self.plan_sequence.clear()
            self.update_plan_display()
            self.plan_start_button.config(text='Start Plan', state='disabled', bg='#0b57d0')
            self.set_buttons_enabled(True)
        self.root.after(0, reset)

    def _on_resize(self, event):
        if event.widget != self.root:
            return
        wrap = max(400, event.width - 100)
        self.text_label.config(wraplength=wrap)
        self.info_text_label.config(wraplength=wrap - 20)

    def run(self):
        self.root.mainloop()


# =============================================================================
# Entry point
# =============================================================================

def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    app = GreetingApp()
    node = GreetingNode(app)
    app.attach_node(node)

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()