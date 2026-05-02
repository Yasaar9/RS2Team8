#!/usr/bin/env python3

import threading
import subprocess
import tkinter as tk
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from gtts import gTTS


class GreetingNode(Node):
    def __init__(self, app):
        super().__init__('greeting_screen_node')
        self.app = app
        self.is_busy = False
        self.stop_requested = False
        self.audio_process = None

        self.declare_parameter(
            'greeting_message',
            'Hello. Welcome to the gallery tour robot.'
        )
        self.declare_parameter(
            'instruction_message_1',
            'To begin, please select an option on the screen.'
        )
        self.declare_parameter(
            'instruction_message_2',
            'You can start a tour, ask for an explanation, type an artefact prompt, or use the selection buttons.'
        )
        self.declare_parameter(
            'instruction_message_3',
            'You can also press stop at any time to interrupt the current response.'
        )
        self.declare_parameter(
            'ready_message',
            'Ready for user input.'
        )

        self.greeting_message = self.get_parameter(
            'greeting_message'
        ).get_parameter_value().string_value

        self.instruction_message_1 = self.get_parameter(
            'instruction_message_1'
        ).get_parameter_value().string_value

        self.instruction_message_2 = self.get_parameter(
            'instruction_message_2'
        ).get_parameter_value().string_value

        self.instruction_message_3 = self.get_parameter(
            'instruction_message_3'
        ).get_parameter_value().string_value

        self.ready_message = self.get_parameter(
            'ready_message'
        ).get_parameter_value().string_value

        self.start_subscription = self.create_subscription(
            Bool,
            '/tour_start',
            self.start_callback,
            10
        )

        self.stop_subscription = self.create_subscription(
            Bool,
            '/tour_stop',
            self.stop_callback,
            10
        )

        self.prompt_subscription = self.create_subscription(
            String,
            '/artifact_prompt',
            self.prompt_callback,
            10
        )

        # IMPROVEMENT 1: Separate topics for Go To and Info actions
        self.goto_subscription = self.create_subscription(
            String,
            '/artifact_goto',
            self.goto_callback,
            10
        )

        self.info_subscription = self.create_subscription(
            String,
            '/artifact_info',
            self.info_callback,
            10
        )

        self.selection_subscription = self.create_subscription(
            String,
            '/artifact_selection',
            self.selection_callback,
            10
        )

        # IMPROVEMENT 2: Subscribe to navigation status feedback
        self.nav_status_subscription = self.create_subscription(
            String,
            '/navigation_status',
            self.nav_status_callback,
            10
        )

        self.navigation_publisher = self.create_publisher(Bool, '/start_navigation', 10)
        self.selected_artifact_publisher = self.create_publisher(String, '/selected_artifact', 10)

        # Publishes Go To destination to navigation_node via /artifact_goto
        self.goto_publisher = self.create_publisher(String, '/artifact_goto', 10)

        # IMPROVEMENT 1: Separate publisher for info-only requests
        self.artifact_info_publisher = self.create_publisher(String, '/artifact_info_request', 10)

        self.artifacts = {
            'modern art': {
                'keywords': ['modern', 'abstract', 'colourful', 'contemporary'],
                'response': 'This is the Modern Art exhibit. It features abstract styles, bold colours, and contemporary themes.',
                'info': (
                    'The Modern Art exhibit showcases works from the 20th and 21st centuries. '
                    'Artists in this collection explore abstract styles, bold use of colour, '
                    'and contemporary social themes. Notable pieces include large-scale canvases '
                    'and mixed media installations.'
                )
            },
            'sculpture': {
                'keywords': ['sculpture', 'statue', '3d', 'stone', 'bronze'],
                'response': 'This is the Sculpture exhibit. It highlights three-dimensional artistic forms and physical craftsmanship.',
                'info': (
                    'The Sculpture exhibit presents three-dimensional works in stone, bronze, and ceramic. '
                    'Each piece demonstrates exceptional craftsmanship, exploring form, texture, and space. '
                    'The collection spans classical to contemporary styles.'
                )
            },
            'portrait': {
                'keywords': ['portrait', 'face', 'person', 'people', 'painting'],
                'response': 'This is the Portrait exhibit. It focuses on human subjects, facial expression, and identity.',
                'info': (
                    'The Portrait exhibit features paintings and photographs of human subjects. '
                    'Works explore identity, emotion, and the relationship between subject and artist. '
                    'The collection includes historical and contemporary portraiture.'
                )
            },
            'historical artefact': {
                'keywords': ['history', 'historical', 'ancient', 'old', 'artifact', 'artefact'],
                'response': 'This is the Historical Artefact exhibit. It presents objects connected to earlier cultures and time periods.',
                'info': (
                    'The Historical Artefact exhibit presents objects from ancient and early modern civilisations. '
                    'Each artefact is accompanied by context explaining its cultural and historical significance. '
                    'Items range from everyday tools to ceremonial objects.'
                )
            }
        }

        self.facilities = {
            'fire exit': 'The nearest emergency fire exit is located toward the marked exit route. Please follow the emergency exit signage.',
            'toilet': 'The toilet is located in the visitor facilities area. Please follow the amenities signage.'
        }

        self.get_logger().info('Greeting screen node ready. Waiting for /tour_start')

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
        # IMPROVEMENT 3: Disable buttons while busy
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
        self.app.cancel_idle_timer()
        self.app.set_status('Listening')
        self.app.set_text('Stopping... Ready for user input.')
        # IMPROVEMENT 3: Re-enable buttons on stop
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

    # IMPROVEMENT 1: New callback for Go To button
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
        self.app.set_text(f'Navigating to {selection.title()}...')
        self.app.set_buttons_enabled(False)
        # IMPROVEMENT 5: Update current destination indicator
        self.app.set_destination(selection.title())
        threading.Thread(target=self.handle_goto, args=(selection,), daemon=True).start()

    # IMPROVEMENT 1: New callback for Info button
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
        self.app.set_text(f'Loading information for {selection.title()}...')
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

    # Navigation status feedback — mirrors /navigation/status from navigation_node
    def nav_status_callback(self, msg):
        status = msg.data.strip()
        self.get_logger().info(f'Navigation status received: {status}')
        self.app.set_nav_status(status)
        # Sync the main status LED with navigation state.
        # Maps every status published by navigation_node to a UI display state.
        status_map = {
            'IDLE':       'Listening',
            'ROTATING':   'Navigating',   # pre-rotation before Nav2 takes over
            'NAVIGATING': 'Navigating',
            'OBSTACLE':   'Navigating',   # still en-route, holding for obstacle
            'REROUTING':  'Rerouting',    # unstuck manoeuvre in progress
            'STUCK':      'Error',        # unstuck failed — no clear direction
            'FAILED':     'Error',
            'REACHED':    'Listening',
        }
        if status in status_map:
            self.app.set_status(status_map[status])
        # Re-enable buttons once navigation is fully done
        if status in ('REACHED', 'FAILED', 'STUCK', 'IDLE'):
            self.is_busy = False
            self.app.set_buttons_enabled(True)

    def run_sequence(self):
        try:
            self.play_beep()
            if self.stop_requested:
                return

            self.app.set_status('Speaking')
            self.app.set_text(self.greeting_message)
            self.speak_text(self.greeting_message)

            if self.stop_requested:
                self.return_to_listening('Stopping... Ready for user input.')
                return

            self.app.set_status('Speaking')
            self.app.set_text(self.instruction_message_1)
            self.speak_text(self.instruction_message_1)

            if self.stop_requested:
                self.return_to_listening('Stopping... Ready for user input.')
                return

            self.app.set_status('Speaking')
            self.app.set_text(self.instruction_message_2)
            self.speak_text(self.instruction_message_2)

            if self.stop_requested:
                self.return_to_listening('Stopping... Ready for user input.')
                return

            self.app.set_status('Speaking')
            self.app.set_text(self.instruction_message_3)
            self.speak_text(self.instruction_message_3)

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

    # Handle Go To — speaks confirmation then sends navigation goal to navigation_node
    def handle_goto(self, selection):
        try:
            if selection in self.artifacts or selection in self.facilities:
                if selection in self.artifacts:
                    speech = f'Navigating to the {selection} exhibit. Please follow the robot.'
                else:
                    speech = f'Guiding you to the {selection}. Please follow the robot.'

                self.app.set_status('Speaking')
                self.app.set_text(speech)
                self.speak_text(speech)

                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return

                # Publish to /artifact_goto — navigation_node maps this to a waypoint
                if self.goto_publisher is not None:
                    goto_msg = String()
                    goto_msg.data = selection
                    self.goto_publisher.publish(goto_msg)
                    self.get_logger().info(f'Published /artifact_goto: {selection}')

                # Also publish /selected_artifact for any other subsystem listeners
                selected_msg = String()
                selected_msg.data = selection
                self.selected_artifact_publisher.publish(selected_msg)

                self.app.set_status('Navigating')
                self.app.set_nav_status(f'Navigating to {selection.title()}...')
                self.return_to_listening(f'Navigating to {selection.title()}. Ready for input.')
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

    # IMPROVEMENT 1 & 4: Handle Info — speaks and displays text only, no navigation
    def handle_info(self, selection):
        try:
            if selection in self.artifacts:
                info_text = self.artifacts[selection]['info']

                # IMPROVEMENT 4: Show info text in the dedicated info panel
                self.app.set_info_panel(selection.title(), info_text)

                self.app.set_status('Speaking')
                self.app.set_text(info_text)
                self.speak_text(info_text)

                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return

                self.return_to_listening(f'Information provided for {selection.title()}. Ready for user input.')
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

    def choose_artifact_from_prompt(self, prompt):
        try:
            prompt_lower = prompt.lower()
            best_match = None
            best_score = 0

            for artifact_name, artifact_info in self.artifacts.items():
                score = sum(
                    1 for keyword in artifact_info['keywords']
                    if keyword in prompt_lower
                )
                if score > best_score:
                    best_score = score
                    best_match = artifact_name

            if best_match is None or best_score == 0:
                error_message = (
                    'Sorry, I could not find a matching artefact for your request. '
                    'Please try prompts such as modern art, sculpture, portrait, or historical artefact.'
                )
                self.app.set_status('Speaking')
                self.app.set_text(error_message)
                self.speak_text(error_message)
                self.return_to_listening('No matching artefact found. Ready for user input.')
                return

            response = (
                f'Based on your request, I recommend the {best_match}. '
                + self.artifacts[best_match]['response']
            )

            selected_msg = String()
            selected_msg.data = best_match
            self.selected_artifact_publisher.publish(selected_msg)

            # IMPROVEMENT 4: Show info in panel on prompt match too
            self.app.set_info_panel(best_match.title(), self.artifacts[best_match]['info'])
            # IMPROVEMENT 5: Update destination
            self.app.set_destination(best_match.title())

            self.app.set_status('Speaking')
            self.app.set_text(response)
            self.speak_text(response)

            if self.stop_requested:
                self.return_to_listening('Stopping... Ready for user input.')
                return

            self.return_to_listening(f'Selected artefact: {best_match}. Ready for user input.')

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
        try:
            if selection in self.artifacts:
                response = self.artifacts[selection]['response']

                selected_msg = String()
                selected_msg.data = selection
                self.selected_artifact_publisher.publish(selected_msg)

                full_response = f'You selected {selection}. {response}'

                self.app.set_status('Speaking')
                self.app.set_text(full_response)
                self.speak_text(full_response)

                if self.stop_requested:
                    self.return_to_listening('Stopping... Ready for user input.')
                    return

                self.return_to_listening(f'Selected artefact: {selection}. Ready for user input.')
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
                stderr=subprocess.DEVNULL
            )
            self.audio_process.wait()
            self.audio_process = None
            if os.path.exists(filename):
                os.remove(filename)
        except Exception as e:
            print(f'Speech error: {e}')


class GreetingApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Tour Guide Robot Interface')
        self.root.geometry('1280x860')
        self.root.configure(bg='#f4f6f8')

        self.idle_after_id = None
        self.all_buttons = []

        # ── Header ──────────────────────────────────────────────────────────
        self.top_frame = tk.Frame(self.root, bg='#0b57d0', height=70)
        self.top_frame.pack(fill='x')

        self.title_label = tk.Label(
            self.top_frame,
            text='Tour Guide Robot Interface',
            font=('Arial', 24, 'bold'),
            fg='white',
            bg='#0b57d0'
        )
        self.title_label.pack(pady=15)

        # ── Status row ──────────────────────────────────────────────────────
        self.status_frame = tk.Frame(self.root, bg='#f4f6f8')
        self.status_frame.pack(pady=(14, 4), fill='x', padx=40)

        self.led_canvas = tk.Canvas(
            self.status_frame,
            width=30, height=30,
            bg='#f4f6f8',
            highlightthickness=0
        )
        self.led_canvas.pack(side='left', padx=10)
        self.led = self.led_canvas.create_oval(5, 5, 25, 25, fill='gray')

        self.status_var = tk.StringVar(value='Status: Waiting')
        self.status_label = tk.Label(
            self.status_frame,
            textvariable=self.status_var,
            font=('Arial', 18, 'bold'),
            fg='#0b57d0',
            bg='#f4f6f8'
        )
        self.status_label.pack(side='left')

        # IMPROVEMENT 5: Current destination indicator
        self.destination_var = tk.StringVar(value='Current destination: None')
        self.destination_label = tk.Label(
            self.status_frame,
            textvariable=self.destination_var,
            font=('Arial', 14),
            fg='#555555',
            bg='#f4f6f8'
        )
        self.destination_label.pack(side='right', padx=20)

        # IMPROVEMENT 2: Navigation status indicator
        self.nav_status_var = tk.StringVar(value='Navigation: Idle')
        self.nav_status_label = tk.Label(
            self.root,
            textvariable=self.nav_status_var,
            font=('Arial', 13, 'italic'),
            fg='#1a6e38',
            bg='#f4f6f8'
        )
        self.nav_status_label.pack(anchor='e', padx=44)

        # ── Main speech display ─────────────────────────────────────────────
        self.text_var = tk.StringVar(value='Waiting for user to press screen...')
        self.text_label = tk.Label(
            self.root,
            textvariable=self.text_var,
            wraplength=1180,
            justify='center',
            font=('Arial', 20),
            bg='white',
            fg='black',
            padx=30,
            pady=22,
            relief='ridge',
            bd=2
        )
        self.text_label.pack(padx=40, pady=(8, 6), fill='x')

        # IMPROVEMENT 4: Artefact info display panel
        self.info_frame = tk.LabelFrame(
            self.root,
            text='Artefact Information',
            font=('Arial', 13, 'bold'),
            bg='#eaf4fb',
            padx=14,
            pady=10
        )
        self.info_frame.pack(fill='x', padx=40, pady=(2, 8))

        self.info_title_var = tk.StringVar(value='')
        self.info_title_label = tk.Label(
            self.info_frame,
            textvariable=self.info_title_var,
            font=('Arial', 14, 'bold'),
            bg='#eaf4fb',
            fg='#0b57d0'
        )
        self.info_title_label.pack(anchor='w')

        self.info_text_var = tk.StringVar(value='Select an artefact and press Info to see details here.')
        self.info_text_label = tk.Label(
            self.info_frame,
            textvariable=self.info_text_var,
            wraplength=1160,
            justify='left',
            font=('Arial', 13),
            bg='#eaf4fb',
            fg='#222222'
        )
        self.info_text_label.pack(anchor='w')

        # ── Prompt row ──────────────────────────────────────────────────────
        self.prompt_frame = tk.Frame(self.root, bg='#f4f6f8')
        self.prompt_frame.pack(pady=(2, 8))

        self.prompt_label = tk.Label(
            self.prompt_frame,
            text='Artefact prompt:',
            font=('Arial', 14, 'bold'),
            bg='#f4f6f8'
        )
        self.prompt_label.pack(side='left', padx=8)

        self.prompt_entry = tk.Entry(
            self.prompt_frame,
            font=('Arial', 14),
            width=45
        )
        self.prompt_entry.pack(side='left', padx=8)

        self.prompt_button = tk.Button(
            self.prompt_frame,
            text='Use Prompt',
            font=('Arial', 14, 'bold'),
            bg='#188038',
            fg='white',
            activebackground='#146c2e',
            activeforeground='white',
            padx=16, pady=6,
            command=self.publish_prompt
        )
        self.prompt_button.pack(side='left', padx=8)
        self.all_buttons.append(self.prompt_button)

        # ── Artefact section — split Go To / Info ───────────────────────────
        self.artifact_section = tk.LabelFrame(
            self.root,
            text='Artefact Selection',
            font=('Arial', 15, 'bold'),
            bg='#f4f6f8',
            padx=15, pady=12
        )
        self.artifact_section.pack(fill='x', padx=40, pady=(4, 6))

        for name, color in [
            ('Modern Art',         '#7b1fa2'),
            ('Sculpture',          '#1565c0'),
            ('Portrait',           '#ef6c00'),
            ('Historical Artefact','#2e7d32'),
        ]:
            self.create_artifact_row(self.artifact_section, name, color)

        # ── Facilities section ──────────────────────────────────────────────
        self.location_section = tk.LabelFrame(
            self.root,
            text='Facilities and Emergency Information',
            font=('Arial', 15, 'bold'),
            bg='#f4f6f8',
            padx=15, pady=12
        )
        self.location_section.pack(fill='x', padx=40, pady=(4, 10))

        self.location_button_frame = tk.Frame(self.location_section, bg='#f4f6f8')
        self.location_button_frame.pack()

        self.create_goto_button(self.location_button_frame, 'Fire Exit', '#c62828')
        self.create_goto_button(self.location_button_frame, 'Toilet',    '#00838f')

        # ── Control buttons ─────────────────────────────────────────────────
        self.button_frame = tk.Frame(self.root, bg='#f4f6f8')
        self.button_frame.pack(pady=14)

        self.start_button = tk.Button(
            self.button_frame,
            text='Start Greeting',
            font=('Arial', 16, 'bold'),
            bg='#0b57d0', fg='white',
            activebackground='#0842a0', activeforeground='white',
            padx=20, pady=10,
            command=self.publish_start
        )
        self.start_button.pack(side='left', padx=10)
        self.all_buttons.append(self.start_button)

        self.stop_button = tk.Button(
            self.button_frame,
            text='Stop',
            font=('Arial', 16, 'bold'),
            bg='#d93025', fg='white',
            activebackground='#b3261e', activeforeground='white',
            padx=20, pady=10,
            command=self.publish_stop
        )
        self.stop_button.pack(side='left', padx=10)
        # Stop button always stays enabled — do NOT add to all_buttons

        # ── ROS publishers (set after attach_node) ──────────────────────────
        self.node = None
        self.start_publisher = None
        self.stop_publisher = None
        self.prompt_publisher = None
        self.goto_publisher = None
        self.info_publisher = None

    # IMPROVEMENT 1: Each artefact gets a "Go To" and an "Info" button
    def create_artifact_row(self, parent, label, color):
        row = tk.Frame(parent, bg='#f4f6f8')
        row.pack(fill='x', pady=4)

        name_label = tk.Label(
            row,
            text=label,
            font=('Arial', 14, 'bold'),
            width=22,
            anchor='w',
            bg='#f4f6f8'
        )
        name_label.pack(side='left', padx=10)

        goto_btn = tk.Button(
            row,
            text='Go To',
            font=('Arial', 13, 'bold'),
            bg=color, fg='white',
            activeforeground='white',
            padx=18, pady=8,
            command=lambda l=label: self.publish_goto(l)
        )
        goto_btn.pack(side='left', padx=6)
        self.all_buttons.append(goto_btn)

        info_btn = tk.Button(
            row,
            text='Info',
            font=('Arial', 13, 'bold'),
            bg='#455a64', fg='white',
            activeforeground='white',
            padx=18, pady=8,
            command=lambda l=label: self.publish_info(l)
        )
        info_btn.pack(side='left', padx=6)
        self.all_buttons.append(info_btn)

    # Go To only (for facilities — info not applicable)
    def create_goto_button(self, parent, label, color):
        btn = tk.Button(
            parent,
            text=f'Go To {label}',
            font=('Arial', 14, 'bold'),
            bg=color, fg='white',
            activeforeground='white',
            padx=18, pady=10,
            command=lambda l=label: self.publish_goto(l)
        )
        btn.pack(side='left', padx=10, pady=5)
        self.all_buttons.append(btn)

    def attach_node(self, node):
        self.node = node
        self.start_publisher  = node.create_publisher(Bool,   '/tour_start',      10)
        self.stop_publisher   = node.create_publisher(Bool,   '/tour_stop',       10)
        self.prompt_publisher = node.create_publisher(String, '/artifact_prompt',  10)
        self.goto_publisher   = node.create_publisher(String, '/artifact_goto',    10)
        self.info_publisher   = node.create_publisher(String, '/artifact_info',    10)

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
                'Rerouting':  '#e65c00',   # amber — unstuck manoeuvre in progress
                'Error':      'red',
            }
            color = color_map.get(status, 'gray')
            if status != 'Listening':
                self.cancel_idle_timer()
            self.led_canvas.itemconfig(self.led, fill=color)
        self.root.after(0, update)

    # IMPROVEMENT 2: Update navigation status label from /navigation_status topic
    def set_nav_status(self, status_text):
        self.root.after(0, lambda: self.nav_status_var.set(f'Navigation: {status_text}'))

    # IMPROVEMENT 3: Enable or disable all interactive buttons
    def set_buttons_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.root.after(0, lambda: [btn.config(state=state) for btn in self.all_buttons])

    # IMPROVEMENT 4: Update info panel with artefact title and description
    def set_info_panel(self, title, text):
        self.root.after(0, lambda: self.info_title_var.set(title))
        self.root.after(0, lambda: self.info_text_var.set(text))

    # IMPROVEMENT 5: Update current destination label
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

    def publish_start(self):
        if self.start_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_text('Processing request...')
            msg = Bool()
            msg.data = True
            self.start_publisher.publish(msg)

    def publish_stop(self):
        if self.stop_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Listening')
            self.set_text('Stopping...')
            self.set_buttons_enabled(True)
            msg = Bool()
            msg.data = True
            self.stop_publisher.publish(msg)

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

    # IMPROVEMENT 1: Publish Go To action
    def publish_goto(self, selection):
        if self.goto_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_destination(selection)
            self.set_text(f'Navigating to {selection}...')
            msg = String()
            msg.data = selection.lower()
            self.goto_publisher.publish(msg)

    # IMPROVEMENT 1: Publish Info action
    def publish_info(self, selection):
        if self.info_publisher is not None:
            self.cancel_idle_timer()
            self.set_status('Processing')
            self.set_text(f'Loading information for {selection}...')
            msg = String()
            msg.data = selection.lower()
            self.info_publisher.publish(msg)

    def run(self):
        self.root.mainloop()


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

    # ros2 run tour_guide_ui greeting_screen_node