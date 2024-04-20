import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk
from threading import Thread

class EmergencyStopPublisher(Node):
    def __init__(self):
        super().__init__('r5sr_emergency_stop_node')
        self.publisher_ = self.create_publisher(Bool, 'emergency_stop', 10)
        self.is_active = False
        
        # tkinter GUI setup
        self.root = tk.Tk()
        self.root.title("R5SR Emergency Stop")
        
        # Configure the main window size
        self.root.geometry('500x400')  # Width x Height
        
        # Setting canvas size
        self.canvas = tk.Canvas(self.root, width=300, height=300, bg='yellow', bd=0, highlightthickness=0)
        self.canvas.pack(pady=20, padx=20)
        
        # Drawing a larger round button
        self.button = self.canvas.create_oval(30, 30, 270, 270, fill='red', outline='darkred')
        # Adding larger, two-line text to the button centered
        self.button_text = self.canvas.create_text(150, 140, text="Emergency", fill="white", font=('Helvetica', '24', 'bold'), anchor='center')
        self.canvas.create_text(150, 180, text="Stop", fill="white", font=('Helvetica', '24', 'bold'), anchor='center')

        self.canvas.tag_bind(self.button, '<Button-1>', self.emergency_stop_callback)
        self.canvas.tag_bind(self.button_text, '<Button-1>', self.emergency_stop_callback)

        # Label for displaying the emergency status message
        self.status_label = tk.Label(self.root, text="", fg="red", font=('Helvetica', '16', 'bold'))
        self.status_label.pack(pady=20)

        # Timer to periodically publish the state
        self.timer = self.create_timer(0.5, self.timer_callback)

        # ROS spinning in a separate thread
        self.ros_thread = Thread(target=self.spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()

    def emergency_stop_callback(self, event):
        self.is_active = not self.is_active
        new_fill = 'darkred' if self.is_active else 'red'
        self.canvas.itemconfig(self.button, fill=new_fill)
        if self.is_active:
            self.status_label.config(text="Emergency Stop Activated")
        else:
            self.status_label.config(text="")

    def timer_callback(self):
        msg = Bool()
        msg.data = self.is_active
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing emergency stop status: {self.is_active}')

    def spin_ros(self):
        rclpy.spin(self)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        rclpy.shutdown()
        self.root.destroy()
        self.ros_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopPublisher()
    node.run()

if __name__ == '__main__':
    main()
