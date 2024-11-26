"""
2024 Metrology Coop Position
Coding prompt

This problem is meant to help you demonstrate competency in python, interaction with hardware, and the ability to design tests.
Hardware syncronization can be a challenge with precision movement.  This is a real problem that was resolved by setting up a hardware trigger to a camera, however we could also resolve it with an algorithm.


You must connect to the linear stage, cause it to continuously move back and forth over a range of 2mm.

Develop a method to make the camera object take an image at equal spaced increments as accurately as possible.

The spacing value should work over a range of 1-100um increments.
You may need to figure out how to adjust the timing of the camera and modify or improve the given classes.
Add in safeguards, show your results over different iterations and write tests for the method.

To connect to the motor go to.
https://software.zaber.com/virtual-device/viewer
you will see the stage after intialization.
select Linear Stages --> LRQ family --> X-LRQ-DE Series -> X-LRQ150AL-DE51

Then copy the cloud link to connect to the device.
c8d2e1c0-3c94-44ec-88f5-0e37e0a4ff16

-Aaron

# This approach works well for velocities below 10 mm/s.
# For higher velocities, due to high acceleration and deceleration during motion start-up and end phases,
# the frame spacing value becomes inconsistent.
# For a more general solution, techniques such as oversampling,
# dynamic frame rate adjustments, or other methods need to be used.
# For an actual camera, we all need to take camera integration and fps time into account.

- Nafiz




"""
import zaber_motion
from zaber_motion import Units, Library, DeviceDbSourceType
from zaber_motion.ascii import Connection, TriggerAction, TriggerCondition
# import numpy as np
import pandas as pd
import time
import datetime
from datetime import datetime
import logging
import matplotlib.pyplot as plt


class ZaberControl(object):
    """
    A wrapper class for zaber motion control
    1. Initialize the object with a connection str and comtype (this will eventually need to be updated to allow for TCP connection)
    Methods
    comtype options : 'serial' used with a comport str like 'COM4', 'iot' used with cloud simulator string.
    connect(): connect to the motor (this is done at init but can be used to reconnect if disconnected)
    home(): home axis
    move_rel(value, velocity:float = 0, accel:float = 0), do a relative move in units, input vel or accel to change params from system default
    move_abs(value, velocity:float = 0, accel:float = 0), same as rel move but absolute position
    pos(): get the position value in degrees units.
    disconnect(): disconnect from the comport.

    """

    def __init__(self, com_port: str, com_type: str = 'serial',
                 db_dir: str = None, scale_factor: int = 1):
        self.com_dat = (com_port, com_type)
        self.is_connected = False
        if db_dir is not None:
            Library.set_device_db_source(DeviceDbSourceType.FILE, db_dir)  # "db\devices-public.sqlite"
        self.connect()
        self.scale_factor = scale_factor

        self.units = {"unit": Units.LENGTH_MILLIMETRES,
                      "velocity_unit": Units.VELOCITY_MILLIMETRES_PER_SECOND,
                      "acceleration_unit": Units.ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED}

        self.move_settings = {"velocity": 0,
                              "acceleration": 0}  # later can be updated to set move/acel values if needed

    def connect(self, alerts: bool = True) -> None:
        # connect to the motor, and initialize axis.
        match self.com_dat[1]:
            case 'serial':
                self.com = Connection.open_serial_port(self.com_dat[0])
            case 'iot':
                self.com = Connection.open_iot(self.com_dat[0])

        if alerts:
            self.com.enable_alerts()
        self.device = self.com.detect_devices()[0]  # need to set up a db for identify devices
        print(f"connected to {self.device}")
        self.axis = self.device.get_axis(1)
        self.triggers = self.device.triggers
        self.triglist = []
        self.triglist.append(self.triggers.get_trigger(1))
        self.is_connected = True

    def disconnect(self):
        self.com.close()
        self.is_connected = False
        print("disconnected from comport")

    def home(self):
        if self.is_connected:
            self.axis.home()
        else:
            print("Not connected to device")

    def pos(self) -> float:
        return self.axis.get_position(self.units["unit"]) / self.scale_factor

    def move_rel(self, position: float, velocity: float = 0, accel: float = 0, wait_until_idle: bool = False):
        # moves by position value, vel and accel can be modified.
        if self.is_connected:
            self.axis.move_relative(position * self.scale_factor, velocity=velocity * self.scale_factor,
                                    acceleration=accel * self.scale_factor, wait_until_idle=wait_until_idle,
                                    **self.units)
        else:
            print("not connected to device")

    def move_abs(self, position: float, velocity: float = 0, accel: float = 0, wait_until_idle: bool = False):
        if self.is_connected:
            self.axis.move_absolute(position * self.scale_factor, velocity=velocity * self.scale_factor,
                                    acceleration=accel * self.scale_factor, wait_until_idle=wait_until_idle,
                                    **self.units)

    def set_cam_trigger_dis(self, distance, trignum: int = 1, ioport: int = 1):
        # set trig condition for low to hi to low every distance increment
        self.triglist[trignum - 1].fire_when_distance_travelled(0, distance * self.scale_factor)
        self.triglist[trignum - 1].on_fire(TriggerAction.A, 0,
                                           "io set do 1 1 schedule 50 0")  # trigeractionA on axis 0, 50ms LOW-HIGH-LOW pulse on digital output 1

    def enable_trigger(self, trignum: int = 1, numbertrigs=None):
        # enable active trigger with optional number of trigs
        if numbertrigs is not None:
            self.triglist[trignum - 1].enable(numbertrigs)
        else:
            self.triglist[trignum - 1].enable()

    def disable_trigger(self, trignum: int = 1):
        # disable trigger condition.
        self.triglist[trignum - 1].disable()

class Camera:
    def __init__(self, zaber: ZaberControl):
        self.zaber = zaber
        self.device = zaber.device
        self.scope = self.device.oscilloscope

    def configure_scope_with_spacing(self, channels, velocity, desired_spacing_microns):
        """
        Configure the oscilloscope with specific channels and a timebase derived
        from the desired spacing and velocity.

        Parameters:
        - channels: A list of tuples specifying channel configurations.
                    Example: [(1, "pos"), (1, "encoder.pos")]
        - velocity: The stage velocity in mm/s.
        - desired_spacing_microns: The desired spacing between data points in µm.
        """

        # Constants
        MAX_ACQUISITION_FREQ_HZ = 10000  # Maximum allowable acquisition frequency in Hz
        MIN_TIMEBASE_MS = 1 / MAX_ACQUISITION_FREQ_HZ * 1000  # Minimum timebase in milliseconds (0.1 ms)


        # Convert spacing to mm
        desired_spacing_mm = desired_spacing_microns / 1000.0

        # Calculate timebase
        timebase = (desired_spacing_mm / velocity) * 1000 # multiplied by 1000 cause timebase unit is in ms

        # Calculate the actual acquisition frequency
        acquisition_freq_hz = 1 / (timebase / 1000.0)  # Convert ms to seconds for frequency

        # Display the resulting acquisition frequency
        print(f"Resulting acquisition frequency: {acquisition_freq_hz:.2f} Hz")

        # Check if acquisition frequency exceeds the maximum allowable threshold
        if acquisition_freq_hz > MAX_ACQUISITION_FREQ_HZ:
            raise ValueError(
                f"The provided combination of spacing ({desired_spacing_microns:.2f} µm) "
                f"and velocity ({velocity:.2f} mm/s) results in an acquisition frequency of {acquisition_freq_hz:.2f} Hz, "
                f"which exceeds the maximum allowed frequency of {MAX_ACQUISITION_FREQ_HZ} Hz. "
                f"Please increase the spacing or reduce the velocity."
            )

        # Clear existing channels to free memory
        self.scope.clear()

        # Add channels
        for channel in channels:
            axis, data_type = channel
            self.scope.add_channel(axis, data_type)

        # Set the calculated timebase
        self.scope.set_timebase(timebase)


# Configure logging for error logging
logging.basicConfig(
    filename='error_log.txt',  # Log file name
    level=logging.ERROR,       # Log only errors and critical issues
    format='%(asctime)s - %(levelname)s - %(message)s',  # Log format with timestamp
    datefmt='%Y-%m-%d %H:%M:%S'  # Date format for logs
)

if __name__ == "__main__":
    cloud_str = "736fd9c5-872f-4712-9264-8991588dca18" # put your cloud str as com_port
    zb = ZaberControl(com_port=cloud_str, com_type='iot')

    cam = Camera(zb)

    zb.home()  # Need to home the system before use.

    while True:  # Loop until valid input
        try:
            # Input start position, end position, velocity, and desired spacing
            start_position = float(input("Enter the start position (mm) (range: 0-150 mm): "))
            end_position = float(input("Enter the end position (mm) (range: 0-150 mm): "))
            velocity = float(input("Enter the velocity (mm/s) (range: 0.000061 - 54 mm/s): "))
            desired_spacing_microns = float(input("Enter the desired spacing (µm) (range: 1-100 um): "))

            # Safeguards based on Linear Stage Model: X-LRQ150AL-DE51

            # Validate start and end positions
            if start_position < 0 or start_position > 150:
                raise ValueError("Start position must be between 0 mm and 150 mm.")
            if end_position < 0 or end_position > 150:
                raise ValueError("End position must be between 0 mm and 150 mm.")

            # Validate velocity
            if velocity < 0.000061 or velocity > 54:
                    raise ValueError("Velocity must be between 0.000061 mm/s and 54 mm/s.")



            # Configure the oscilloscope with spacing
            channels = [(1, "pos"), (1, "encoder.pos")]  # Specify channels explicitly
            cam.configure_scope_with_spacing(channels, velocity, desired_spacing_microns)

            # Break the loop if configuration is successful
            break

        except ValueError as e:
            # Log the error to the error log
            logging.error(str(e))

            # Print the error message for the user
            print()
            print(f"Error: {e}")
            print("Please try again with valid values.")

    # Continue with motion and data capture after valid input
    axis = zb.device.get_axis(1)

    # Move from start position to end position at specified velocity
    zb.move_abs(start_position, velocity, 0, wait_until_idle=True)  # Move to start position and wait
    print(f"Position after first move: {zb.pos() * 1000:.2f} µm")  # Print position in microns

    # Start capturing data during motion
    # scope is started before motion, otherwise it misses some initial frames due to lag
    cam.scope.start()

    zb.move_abs(end_position, velocity, 0, wait_until_idle=False)  # Start move to end position

    # Wait for motion to complete
    axis.wait_until_idle()
    cam.scope.stop()

    # Read and process the scope data
    channels_data = cam.scope.read()  # Read captured channel data
    pos_data_points = channels_data[0].get_data(Units.LENGTH_MILLIMETRES)  # Extract position data
    encoder_pos_data_points = channels_data[1].get_data(Units.LENGTH_MILLIMETRES)  # Extract encoder data

    # Convert all units to microns (1 mm = 1000 µm)
    pos_data_points_microns = [round(x * 1000, 1) for x in pos_data_points]
    encoder_pos_data_points_microns = [round(x * 1000, 1) for x in encoder_pos_data_points]

    # Generate frame numbers
    frame_numbers = list(range(1, len(pos_data_points_microns) + 1))

    # Store data in a Pandas DataFrame
    df = pd.DataFrame({
        "Frame Number": frame_numbers,
        "Position (µm)": pos_data_points_microns,
        "Encoder Position (µm)": encoder_pos_data_points_microns,
    })

    # we need to trim the extra frames which was captured by the scope before the stage started moving towards end position

    # Define a tolerance for comparison (in µm)
    tolerance = 0.1

    # Find the last occurrence of the start position within tolerance
    start_index = df[
        (df["Encoder Position (µm)"] >= start_position * 1000 - tolerance) &
        (df["Encoder Position (µm)"] <= start_position * 1000 + tolerance)
        ].last_valid_index()

    # Find the first occurrence of the end position within tolerance
    end_index = df[
        (df["Encoder Position (µm)"] >= end_position * 1000 - tolerance) &
        (df["Encoder Position (µm)"] <= end_position * 1000 + tolerance)
        ].first_valid_index()

    # Ensure valid indices are found before trimming
    if start_index is None or end_index is None:
        error_message = f"Start position {start_position} µm or end position {end_position} µm not found in the data."
        logging.error(error_message)
        raise ValueError(error_message)

    # Trim the DataFrame to keep only rows between the identified indices
    df_trimmed = df.loc[start_index:end_index]

    # Save the trimmed DataFrame to a CSV file
    output_file = "motion_data_microns.csv"
    df_trimmed.to_csv(output_file, index=False)

    # Print the trimmed DataFrame
    print(df_trimmed)

    # Trim the DataFrame to keep only rows between the identified indices
    df_trimmed = df.loc[start_index:end_index].copy()  # Add .copy() to create an independent copy

    # Visualization Section for performance assessment and troubleshooting

    def plot_graph(x, y, title, xlabel, ylabel, line_label=None, axhline=None, color='blue'):
        """
        A reusable function for plotting graphs with consistent styling.

        Parameters:
        - x: Data for the x-axis.
        - y: Data for the y-axis.
        """
        plt.figure(figsize=(10, 6))
        plt.plot(x, y, marker='o', linestyle='-', markersize=3, color=color)
        plt.title(title, fontsize=14)
        plt.xlabel(xlabel, fontsize=12)
        plt.ylabel(ylabel, fontsize=12)
        if axhline:
            plt.axhline(y=axhline, color='r', linestyle='--', label=line_label)
        if line_label:
            plt.legend()
        plt.grid(True)
        plt.show()


    # Plot 1: Position (µm) vs. Frame Number
    plot_graph(
        df_trimmed["Frame Number"],
        df_trimmed["Position (µm)"],
        "Position (µm) vs. Frame Number",
        "Frame Number",
        "Position (µm)"
    )

    # Calculate spacing increments
    df_trimmed["Increment (µm)"] = df_trimmed["Position (µm)"].diff()

    # Plot 2: Incremental Spacing (µm) vs. Frame Number
    plot_graph(
        df_trimmed["Frame Number"][1:],
        df_trimmed["Increment (µm)"][1:],
        "Incremental Spacing (µm) vs. Frame Number",
        "Frame Number",
        "Incremental Spacing (µm)",
        line_label=f"Target Spacing ({desired_spacing_microns} µm)",
        axhline=desired_spacing_microns,
        color='orange'
    )









