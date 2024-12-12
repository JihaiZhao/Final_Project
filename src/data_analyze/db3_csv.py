import sqlite3
import csv
import rclpy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from datetime import datetime

def db3_to_csv(db_file, csv_file):
    # Initialize ROS 2 Python client library
    rclpy.init()

    # Connect to the database
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    # Fetch all rows from the messages table
    cursor.execute("SELECT timestamp, data FROM messages")
    
    rows = cursor.fetchall()
    
    # Check if we have rows and print them for debugging
    if not rows:
        print("No data found in the messages table.")
        return

    print(f"Found {len(rows)} rows in the messages table.")

    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write the header row
        writer.writerow(['time', 'header', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z'])

        # Process each row
        for row in rows:
            timestamp, data_blob = row
            
            # Convert timestamp to the required format
            timestamp = datetime.fromtimestamp(timestamp / 1e9).strftime('%Y/%m/%d %H:%M:%S.%f')

            # Check if the BLOB is empty
            if not data_blob:
                print("Empty BLOB data, skipping...")
                continue

            # Attempt to deserialize the BLOB data
            try:
                # Deserialize the message
                message = deserialize_message(data_blob, WrenchStamped)
            except Exception as e:
                print(f"Error decoding BLOB data: {e}")
                continue  # Skip this entry if there's an error

            # Extract 'header' and 'wrench'
            header = message.header
            wrench = message.wrench

            # Format 'header' as a string
            header_str = {
                'stamp': {'secs': header.stamp.sec, 'nsecs': header.stamp.nanosec},
                'frame_id': header.frame_id
            }

            # Extract wrench components
            force_x = wrench.force.x
            force_y = wrench.force.y
            force_z = wrench.force.z
            torque_x = wrench.torque.x
            torque_y = wrench.torque.y
            torque_z = wrench.torque.z

            # Write the formatted data to CSV
            writer.writerow([
                timestamp,
                str(header_str),  # Convert header dict to string
                force_x,
                force_y,
                force_z,
                torque_x,
                torque_y,
                torque_z
            ])

    # Close the connection
    conn.close()

    # Shutdown ROS 2 Python client library
    rclpy.shutdown()

# Example usage
db3_to_csv('/home/jihai/Jihai/FT_ws/impedance_force_x/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/impedance_force_x_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/impedance_force_y/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/impedance_force_y_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/impedance_force_z/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/impedance_force_z_ft.csv')

db3_to_csv('/home/jihai/Jihai/FT_ws/damping_x/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/damping_x_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/damping_y/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/damping_y_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/damping_z/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/damping_z_ft.csv')

db3_to_csv('/home/jihai/Jihai/FT_ws/friction_compensation_x/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/friction_compensation_x_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/friction_compensation_y/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/friction_compensation_y_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/friction_compensation_z/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/friction_compensation_z_ft.csv')

db3_to_csv('/home/jihai/Jihai/FT_ws/white_light_x/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/white_light_x_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/white_light_y/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/white_light_y_ft.csv')
db3_to_csv('/home/jihai/Jihai/FT_ws/white_light_z/my_bag_0.db3', '/home/jihai/Jihai/final_project_data/white_light_z_ft.csv')