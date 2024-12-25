import tkinter as tk
import socket
import threading
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.action_chains import ActionChains
import time
import json

class GestureSyncMusicController:
    def __init__(self, playlist_url, tcp_port=8002):
        # YouTube Setup
        options = webdriver.ChromeOptions()
        options.add_argument("--start-maximized")
        options.add_experimental_option("detach", True)
        
        service = Service(ChromeDriverManager().install())
        self.driver = webdriver.Chrome(service=service, options=options)
        self.driver.get(playlist_url)
        
        # State tracking
        self.is_playing = False
        self.current_song_index = 0
        
        # Volume Control Attributes
        self.volume_changing = False
        self.volume_control_mode = None
        self.initial_volume = 50  # Default initial volume
        self.volume_sensitivity = 10/9  # abt 1% volume change per degree
        
        # Network Setup
        self.TCP_IP = '192.168.134.66'  # Listen on all available interfaces
        self.TCP_PORT = tcp_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.TCP_IP, self.TCP_PORT))
        self.socket.listen(1)
        
        # UI Setup
        self.setup_ui()
        
        # Start TCP Listener
        self.start_tcp_listener()

    def setup_ui(self):
        self.root = tk.Tk()
        self.root.title("GestureSync Music Controller")
        self.root.geometry("400x300")
        
        # Status Display
        self.status_label = tk.Label(self.root, text="Waiting for STM32 Connection...", 
                                     font=("Arial", 12), wraplength=350)
        self.status_label.pack(pady=20)
        
        # Connection Status
        self.connection_status = tk.Label(self.root, text="", 
                                          font=("Arial", 10), fg="green")
        self.connection_status.pack(pady=10)

    def start_tcp_listener(self):
        # Start TCP listening in a separate thread
        tcp_thread = threading.Thread(target=self.listen_for_commands)
        tcp_thread.daemon = True
        tcp_thread.start()

    def listen_for_commands(self):
        while True:
            try:
                # Wait for client connection
                client_socket, addr = self.socket.accept()
                self.update_status(f"Connected to: {addr}")
                msg_i = 'Connected'
                client_socket.send(msg_i.encode())
                
                while True:
                    # Receive data
                    if self.volume_changing:
                        data = client_socket.recv(1024)
                        data_int = int.from_bytes(data[0:-1],byteorder= 'little', signed=True)
                        if data_int <= 100 and data_int >= -100 :
                            data = data_int
                            print(data)
                        else :
                            data = data.decode("utf-8")
                            data = data[0:-1]
                            print(data)
                    else:
                        data = client_socket.recv(1024).decode("utf-8")
                        data = data[0:-1]
                        print(data)
                    
                    # Process received command
                    self.process_gesture_command(data)
                    msg_f = 'Completed'
                    client_socket.send(msg_f.encode())
            
            except Exception as e:
                self.update_status(f"Connection Error: {e}")
                time.sleep(2)

    def process_gesture_command(self, command):
        """
        Map STM32 gestures to music control actions
        Supported Gestures:
        - 'Left': Previous Song
        - 'Right': Next Song
        - 'Down': Pause
        - 'Up': Play/Resume
        - 'volume_start': Begin Volume Control
        - 'volume_stop': End Volume Control
        """
        try:
            # Volume Control Start Signal
            if command == 'volume_start':
                self.volume_control_mode = 'relative'
                self.volume_changing = True
                # Retrieve initial volume
                self.initial_volume = self.get_current_volume()
                self.update_status("Relative Volume Control Started")
                return
            
            # Volume Control Stop Signal
            elif command == "volume_stop":
                self.volume_control_mode = None
                self.volume_changing = False
                self.update_status("Volume Control Stopped")
                return
            
            # Check if in volume control mode
            if self.volume_changing and self.volume_control_mode == 'relative':
                try:
                    # Convert received rotation to volume change
                    rotation = float(command)
                    self.adjust_relative_volume(rotation)
                    return
                except ValueError:
                    self.update_status(f"Invalid volume input: {command}")
            
            # Parse command for standard gestures
            gesture = command.strip().lower()

            # Map gestures to actions
            gesture_map = {
                'left': self.previous_song,
                'right': self.next_song,
                'down': self.pause,
                'up': self.play
            }
            
            # Execute corresponding action
            action = gesture_map.get(gesture)
            if action:
                action()
                self.update_status(f"Executed: {gesture}")
            else:
                self.update_status(f"Unknown Gesture: {gesture}")
        
        except Exception as e:
            self.update_status(f"Command Processing Error: {e}")

    def adjust_relative_volume(self, rotation):
        try:
            # Ignore very small rotations to prevent noise
            if abs(rotation) < 0.1:
                return
            
            # Current volume retrieval
            current_volume = self.get_current_volume()
            
            # Volume change calculation
            volume_change = rotation * self.volume_sensitivity
            
            # Calculate new volume
            new_volume = max(0, min(100, current_volume + volume_change))
            
            # JavaScript to set volume
            volume_script = f"""
            var video = document.querySelector('video');
            if (video) {{
                video.volume = {new_volume / 100};
                return video.volume * 100;
            }}
            return null;
            """
            
            # Execute volume adjustment
            result = self.driver.execute_script(volume_script)
            
            if result is not None:
                actual_volume = round(result)
                change_direction = "increased" if volume_change > 0 else "decreased"
                self.update_status(f"Volume {change_direction} to {actual_volume}%")
            else:
                self.update_status("Failed to adjust volume")
        
        except Exception as e:
            self.update_status(f"Volume Adjustment Error: {e}")

    def get_current_volume(self):
        try:
            volume_script = """
            var video = document.querySelector('video');
            return video ? video.volume * 100 : null;
            """
            
            current_volume = self.driver.execute_script(volume_script)
            
            return round(current_volume) if current_volume is not None else 50
        
        except Exception as e:
            self.update_status(f"Volume Retrieval Error: {e}")
            return 50  # Default safe volume


    def update_status(self, message):
        # Update UI status label
        def update():
            self.status_label.config(text=message)
        self.root.after(0, update)

    def play(self):
        try:
            # Multiple methods to play
            methods = [
                lambda: self.driver.execute_script("document.querySelector('video').play()"),
                lambda: ActionChains(self.driver).send_keys(Keys.SPACE).perform(),
            ]
            
            for method in methods:
                try:
                    method()
                    self.is_playing = True
                    self.update_status("Music Resumed")
                    return
                except Exception:
                    continue
        except Exception as e:
            self.update_status(f"Play Error: {e}")

    def pause(self):
        try:
            # Multiple methods to pause
            methods = [
                lambda: self.driver.execute_script("document.querySelector('video').pause()"),
                lambda: ActionChains(self.driver).send_keys(Keys.SPACE).perform(),
            ]
            
            for method in methods:
                try:
                    method()
                    self.is_playing = False
                    self.update_status("Music Paused")
                    return
                except Exception:
                    continue
        except Exception as e:
            self.update_status(f"Pause Error: {e}")

    def next_song(self):
        try:
            # Multiple methods for next song
            methods = [
                lambda: self.find_and_click_playlist_next(),
                lambda: self.driver.find_element(By.CLASS_NAME, 'ytp-next-button').click(),
                lambda: ActionChains(self.driver).send_keys(Keys.SHIFT + Keys.RIGHT).perform()
            ]
            
            for method in methods:
                try:
                    method()
                    self.update_status("Next Song")
                    return
                except Exception:
                    continue
        except Exception as e:
            self.update_status(f"Next Song Error: {e}")

    def previous_song(self):
        try:
            # Multiple methods for previous song
            methods = [
                lambda: self.find_and_click_playlist_previous(),
                lambda: self.driver.execute_script("document.querySelector('video').currentTime = 0"),
                lambda: ActionChains(self.driver).send_keys(Keys.SHIFT + Keys.LEFT).perform()
            ]
            
            for method in methods:
                try:
                    method()
                    self.update_status("Previous Song")
                    return
                except Exception:
                    continue
        except Exception as e:
            self.update_status(f"Previous Song Error: {e}")

    def find_and_click_playlist_next(self):
        playlist_next_selector = "a.ytd-playlist-panel-video-renderer[href]"
        playlist_videos = self.driver.find_elements(By.CSS_SELECTOR, playlist_next_selector)
        
        if self.current_song_index + 1 < len(playlist_videos):
            self.current_song_index += 1
            playlist_videos[self.current_song_index].click()

    def find_and_click_playlist_previous(self):
        playlist_prev_selector = "a.ytd-playlist-panel-video-renderer[href]"
        playlist_videos = self.driver.find_elements(By.CSS_SELECTOR, playlist_prev_selector)
        
        if self.current_song_index > 0:
            self.current_song_index -= 1
            playlist_videos[self.current_song_index].click()

    def run(self):
        self.root.mainloop()

def main():
    playlist_url = "https://www.youtube.com/watch?v=hmUyEDG7Jy0&list=RDQM_imuc_M9hgc&start_radio=1"
    controller = GestureSyncMusicController(playlist_url)
    controller.run()

if __name__ == "__main__":
    main()